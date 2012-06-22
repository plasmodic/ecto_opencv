/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/rgbd/rgbd.hpp>

#include <iostream>

#if defined(HAVE_EIGEN) && EIGEN_WORLD_VERSION == 3
#define HAVE_EIGEN3_HERE
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/Dense>
#endif

#define SHOW_DEBUG_IMAGES 0
#define IS_RGBD 1

using namespace cv;

inline static
void computeProjectiveMatrix(const Mat& ksi, Mat& Rt)
{
    CV_Assert(ksi.size() == Size(1,6) && ksi.type() == CV_64FC1);

#ifdef HAVE_EIGEN3_HERE
    const double* ksi_ptr = reinterpret_cast<const double*>(ksi.ptr(0));
    Eigen::Matrix<double,4,4> twist, g;
    twist << 0.,          -ksi_ptr[2], ksi_ptr[1],  ksi_ptr[3],
             ksi_ptr[2],  0.,          -ksi_ptr[0], ksi_ptr[4],
             -ksi_ptr[1], ksi_ptr[0],  0,           ksi_ptr[5],
             0.,          0.,          0.,          0.;
    g = twist.exp();


    eigen2cv(g, Rt);
#else
    std::cout << "!!!!! WARNING !!!!!: check computeProjectiveMatrix when there is not eigen library, because it gives less accurate pose of the camera." << std::endl;
    Rt = Mat::eye(4, 4, CV_64FC1);

    Mat R = Rt(Rect(0,0,3,3));
    Mat rvec = ksi.rowRange(0,3);

    Rodrigues(rvec, R);

    Rt.at<double>(0,3) = ksi.at<double>(3);
    Rt.at<double>(1,3) = ksi.at<double>(4);
    Rt.at<double>(2,3) = ksi.at<double>(5);
#endif
}

static inline
void preprocessDepth(Mat depth0, Mat depth1,
                     const Mat& validMask0, const Mat& validMask1,
                     float minDepth, float maxDepth)
{
    CV_DbgAssert(depth0.size() == depth1.size());

    for(int y = 0; y < depth0.rows; y++)
    {
        for(int x = 0; x < depth0.cols; x++)
        {
            float& d0 = depth0.at<float>(y,x);
            if(!cvIsNaN(d0) && (d0 > maxDepth || d0 < minDepth || d0 <= 0 || (!validMask0.empty() && !validMask0.at<uchar>(y,x))))
                d0 = std::numeric_limits<float>::quiet_NaN();

            float& d1 = depth1.at<float>(y,x);
            if(!cvIsNaN(d1) && (d1 > maxDepth || d1 < minDepth || d1 <= 0 || (!validMask1.empty() && !validMask1.at<uchar>(y,x))))
                d1 = std::numeric_limits<float>::quiet_NaN();
        }
    }
}

static
void depth2Cloud(const Mat& depth, Mat& cloud, const Mat& cameraMatrix)
{
    CV_Assert(cameraMatrix.type() == CV_64FC1);
    const double inv_fx = 1.f/cameraMatrix.at<double>(0,0);
    const double inv_fy = 1.f/cameraMatrix.at<double>(1,1);
    const double ox = cameraMatrix.at<double>(0,2);
    const double oy = cameraMatrix.at<double>(1,2);
    cloud.create(depth.size(), CV_32FC3);
    for(int y = 0; y < cloud.rows; y++)
    {
        Point3f* cloud_ptr = reinterpret_cast<Point3f*>(cloud.ptr(y));
        const float* depth_prt = reinterpret_cast<const float*>(depth.ptr(y));
        for(int x = 0; x < cloud.cols; x++)
        {
            float z = depth_prt[x];
            cloud_ptr[x].x = (float)((x - ox) * z * inv_fx);
            cloud_ptr[x].y = (float)((y - oy) * z * inv_fy);
            cloud_ptr[x].z = z;
        }
    }
}

static
bool solveSystem(const Mat& AtA, const Mat& AtB, double detThreshold, Mat& x)
{
    double det = cv::determinant(AtA);

    if(fabs (det) < detThreshold || cvIsNaN(det) || cvIsInf(det))
        return false;

    cv::solve(AtA, AtB, x, DECOMP_CHOLESKY);

    return true;
}

inline static
void computeC(double* C, double dIdx, double dIdy, const Point3f& p3d, double fx, double fy)
{
    double invz  = 1. / p3d.z,
           v0 = dIdx * fx * invz,
           v1 = dIdy * fy * invz,
           v2 = -(v0 * p3d.x + v1 * p3d.y) * invz;

    C[0] = -p3d.z * v1 + p3d.y * v2;
    C[1] =  p3d.z * v0 - p3d.x * v2;
    C[2] = -p3d.y * v0 + p3d.x * v1;
    C[3] = v0;
    C[4] = v1;
    C[5] = v2;
}

static inline
void set2shorts(int& dst, int short_v1, int short_v2)
{
    unsigned short* ptr = reinterpret_cast<unsigned short*>(&dst);
    ptr[0] = static_cast<unsigned short>(short_v1);
    ptr[1] = static_cast<unsigned short>(short_v2);
}

static inline
void get2shorts(int src, int& short_v1, int& short_v2)
{
    typedef union { int vint32; unsigned short vuint16[2]; } s32tou16;
    const unsigned short* ptr = (reinterpret_cast<s32tou16*>(&src))->vuint16;
    short_v1 = ptr[0];
    short_v2 = ptr[1];
}

static
int computeCorresp(const Mat& K, const Mat& K_inv, const Mat& Rt,
                   const Mat& depth0, const Mat& depth1, const Mat& mask1, float maxDepthDiff,
                   Mat& corresps)
{
    CV_Assert(K.type() == CV_64FC1);
    CV_Assert(K_inv.type() == CV_64FC1);
    CV_Assert(Rt.type() == CV_64FC1);

    corresps.create(depth1.size(), CV_32SC1);

    Mat R = Rt(Rect(0,0,3,3)).clone();

    Mat KRK_inv = K * R * K_inv;
    const double * KRK_inv_ptr = reinterpret_cast<const double *>(KRK_inv.ptr());

    Mat Kt = Rt(Rect(3,0,1,3)).clone();
    Kt = K * Kt;
    const double * Kt_ptr = reinterpret_cast<const double *>(Kt.ptr());

    Rect r(0, 0, depth1.cols, depth1.rows);

    corresps = Scalar(-1);
    int correspCount = 0;
    for(int v1 = 0; v1 < depth1.rows; v1++)
    {
        for(int u1 = 0; u1 < depth1.cols; u1++)
        {
            float d1 = depth1.at<float>(v1,u1);
            if(!cvIsNaN(d1) && mask1.at<uchar>(v1,u1))
            {
                float transformed_d1 = (float)(d1 * (KRK_inv_ptr[6] * u1 + KRK_inv_ptr[7] * v1 + KRK_inv_ptr[8]) + Kt_ptr[2]);
                int u0 = cvRound((d1 * (KRK_inv_ptr[0] * u1 + KRK_inv_ptr[1] * v1 + KRK_inv_ptr[2]) + Kt_ptr[0]) / transformed_d1);
                int v0 = cvRound((d1 * (KRK_inv_ptr[3] * u1 + KRK_inv_ptr[4] * v1 + KRK_inv_ptr[5]) + Kt_ptr[1]) / transformed_d1);

                if(r.contains(Point(u0,v0)))
                {
                    float d0 = depth0.at<float>(v0,u0);
                    if(!cvIsNaN(d0) && std::abs(transformed_d1 - d0) <= maxDepthDiff)
                    {
                        int c = corresps.at<int>(v0,u0);
                        if(c != -1)
                        {
                            int exist_u1, exist_v1;
                            get2shorts(c, exist_u1, exist_v1);

                            float exist_d1 = (float)(depth1.at<float>(exist_v1,exist_u1) * (KRK_inv_ptr[6] * exist_u1 + KRK_inv_ptr[7] * exist_v1 + KRK_inv_ptr[8]) + Kt_ptr[2]);

                            if(transformed_d1 > exist_d1)
                                continue;
                        }
                        else
                            correspCount++;

                        set2shorts(corresps.at<int>(v0,u0), u1, v1);
                    }
                }
            }
        }
    }

    return correspCount;
}


static
void buildPyramids(const Mat& image0, const Mat& image1,
                   const Mat& depth0, const Mat& depth1,
                   const Mat& cameraMatrix, int sobelSize, double sobelScale,
                   const vector<float>& minGradMagnitudes,
                   vector<Mat>& pyramidImage0, vector<Mat>& pyramidDepth0,
                   vector<Mat>& pyramidImage1, vector<Mat>& pyramidDepth1,
                   vector<Mat>& pyramid_dI_dx1, vector<Mat>& pyramid_dI_dy1,
                   vector<Mat>& pyramidVertices0, vector<Mat>& pyramidVertices1,
                   vector<Mat>& pyramidNormals1,
                   vector<Mat>& pyramidTexturedMask1, vector<Mat>& pyramidNormalMask1,
                   vector<Mat>& pyramidCameraMatrix,
                   vector<Ptr<RgbdNormals> >& normalComputers)
{

    const int pyramidLevels = (int)minGradMagnitudes.size();

    pyramidCameraMatrix.reserve(pyramidLevels);
    Mat cameraMatrix_dbl;
    cameraMatrix.convertTo(cameraMatrix_dbl, CV_64FC1);
    for(int i = 0; i < pyramidLevels; i++)
    {
        Mat levelCameraMatrix = i == 0 ? cameraMatrix_dbl : 0.5f * pyramidCameraMatrix[i-1];
        levelCameraMatrix.at<double>(2,2) = 1.;
        pyramidCameraMatrix.push_back(levelCameraMatrix);
    }

    buildPyramid(image0, pyramidImage0, pyramidLevels - 1);
    buildPyramid(image1, pyramidImage1, pyramidLevels - 1);

    buildPyramid(depth0, pyramidDepth0, pyramidLevels - 1);
    buildPyramid(depth1, pyramidDepth1, pyramidLevels - 1);

    pyramid_dI_dx1.resize(pyramidLevels);
    pyramid_dI_dy1.resize(pyramidLevels);
    pyramidVertices0.resize(pyramidLevels);
    pyramidVertices1.resize(pyramidLevels);
    pyramidNormals1.resize(pyramidLevels);
    pyramidTexturedMask1.resize(pyramidLevels);
    pyramidNormalMask1.resize(pyramidLevels);

    if(normalComputers.empty())
        normalComputers.resize(pyramidLevels);

    for(size_t i = 0; i < pyramidImage1.size(); i++)
    {
        Sobel(pyramidImage1[i], pyramid_dI_dx1[i], CV_16S, 1, 0, sobelSize);
        Sobel(pyramidImage1[i], pyramid_dI_dy1[i], CV_16S, 0, 1, sobelSize);

        const Mat& dx = pyramid_dI_dx1[i];
        const Mat& dy = pyramid_dI_dy1[i];

        depth2Cloud(pyramidDepth0[i], pyramidVertices0[i], pyramidCameraMatrix[i]);
        depth2Cloud(pyramidDepth1[i], pyramidVertices1[i], pyramidCameraMatrix[i]);

        const Mat& levelDepth1 = pyramidDepth1[i];
        const Mat& levelVertices1 = pyramidVertices1[i];
        Mat& normals1 = pyramidNormals1[i];
        normals1.create(levelVertices1.size(), CV_32FC3);
        normals1 = Scalar::all(std::numeric_limits<float>::quiet_NaN());

        Mat texturedMask1 = Mat(levelVertices1.size(), CV_8UC1, Scalar::all(0));
        Mat normalMask1 = Mat(levelVertices1.size(), CV_8UC1, Scalar::all(0));

        const Mat& K = pyramidCameraMatrix[i];

        if(normalComputers[i] == 0)
            normalComputers[i] = new RgbdNormals(levelDepth1.rows, levelDepth1.cols, levelDepth1.depth(), K, 3);


        Mat points3d;
        depthTo3d(levelDepth1, K, points3d);
        normals1 = (*normalComputers[i])(points3d);
        CV_Assert(normals1.type() ==CV_32FC3);
        for(int y = 0; y < normals1.rows; y++)
        {
            for(int x = 0; x < normals1.cols; x++)
            {
                if(!cvIsNaN(normals1.at<Point3f>(y,x).x) &&
                   !cvIsNaN(normals1.at<Point3f>(y,x).y) &&
                   !cvIsNaN(normals1.at<Point3f>(y,x).z))
                    normalMask1.at<uchar>(y,x) = 255;
            }
        }
        const float minScalesGradMagnitude2 = (float)((minGradMagnitudes[i] * minGradMagnitudes[i]) / (sobelScale * sobelScale));
        for(int y = 0; y < levelDepth1.rows; y++)
        {
            for(int x = 0; x < levelDepth1.cols; x++)
            {
                float m2 = (float)(dx.at<short>(y,x)*dx.at<short>(y,x) + dy.at<short>(y,x)*dy.at<short>(y,x));
                if(m2 >= minScalesGradMagnitude2)
                    texturedMask1.at<uchar>(y,x) = 255;
            }
        }

        pyramidTexturedMask1[i] = texturedMask1;
        pyramidNormalMask1[i] = normalMask1;
    }
}

static 
void computeAB(const Mat& image0, const Mat&  cloud0,
               const Mat& image1, const Mat& dI_dx1, const Mat& dI_dy1,
               const Mat& corresps, int correspsCount,
               double fx, double fy, double sobelScale,
               Mat& A, Mat& B)
{
    Mat C(correspsCount, 6, CV_64FC1);
    Mat dI_dt(correspsCount, 1, CV_64FC1);

    double sigma = 0;
    int pointCount = 0;
    for(int v0 = 0; v0 < corresps.rows; v0++)
    {
        for(int u0 = 0; u0 < corresps.cols; u0++)
        {
            if(corresps.at<int>(v0,u0) != -1)
            {
                int u1, v1;
                get2shorts(corresps.at<int>(v0,u0), u1, v1);

                double diff = static_cast<double>(image0.at<uchar>(v0,u0)) -
                               static_cast<double>(image1.at<uchar>(v1,u1));
                sigma += diff * diff;
                pointCount++;
            }
        }
    }
    sigma = std::sqrt(sigma/pointCount);

    pointCount = 0;
    for(int v0 = 0; v0 < corresps.rows; v0++)
    {
        for(int u0 = 0; u0 < corresps.cols; u0++)
        {
            if(corresps.at<int>(v0,u0) != -1)
            {
                int u1, v1;
                get2shorts(corresps.at<int>(v0,u0), u1, v1);

                double diff = static_cast<double>(image0.at<uchar>(v0,u0)) - static_cast<double>(image1.at<uchar>(v1,u1));

                double w = sigma + std::abs(diff);
                w = w > DBL_EPSILON ? 1./w : 1.;

                computeC((double*)C.ptr(pointCount),
                         w * sobelScale * dI_dx1.at<short int>(v1,u1),
                         w * sobelScale * dI_dy1.at<short int>(v1,u1),
                         cloud0.at<Point3f>(v0,u0), fx, fy);

                dI_dt.at<double>(pointCount) = w * diff;
                pointCount++;
            }
        }
    }

    A = C;
    B = dI_dt;
}

inline static
void computeC(double* C, const Point3f& p0, const Vec3f& n1)
{
    C[0] = -p0.z * n1[1] + p0.y * n1[2];
    C[1] =  p0.z * n1[0] - p0.x * n1[2];
    C[2] = -p0.y * n1[0] + p0.x * n1[1];
    C[3] = n1[0];
    C[4] = n1[1];
    C[5] = n1[2];
}

static
void computeAB(const Mat& levelCloud0, const Mat& Rt,
               const Mat& levelCloud1, const Mat& levelNormals1,
               const Mat& corresps, int correspsCount,
               Mat& A, Mat& B)
{
    Mat C(correspsCount, 6, CV_64FC1);
    Mat dV_dt(correspsCount, 1, CV_64FC1);

    CV_Assert(Rt.type() == CV_64FC1);
    const double * Rt_ptr = reinterpret_cast<const double*>(Rt.data);

    double sigma = 0;
    int pointCount = 0;
    for(int v0 = 0; v0 < corresps.rows; v0++)
    {
        for(int u0 = 0; u0 < corresps.cols; u0++)
        {
            if(corresps.at<int>(v0,u0) != -1)
            {
                int u1, v1;
                get2shorts(corresps.at<int>(v0,u0), u1, v1);

                Point3f p0 = levelCloud0.at<Point3f>(v0, u0), tp0; // tp0 - transformed p0
                tp0.x = p0.x * Rt_ptr[0] + p0.y * Rt_ptr[1] + p0.z * Rt_ptr[2] + Rt_ptr[3];
                tp0.y = p0.x * Rt_ptr[4] + p0.y * Rt_ptr[5] + p0.z * Rt_ptr[6] + Rt_ptr[7];
                tp0.z = p0.x * Rt_ptr[8] + p0.y * Rt_ptr[9] + p0.z * Rt_ptr[10] + Rt_ptr[11];

                Vec3f n1 = levelNormals1.at<Vec3f>(v1, u1);
                Point3f v = levelCloud1.at<Point3f>(v1,u1) - tp0;
                double diff = n1[0] * v.x + n1[1] * v.y + n1[2] * v.z;
                sigma += diff * diff;
                pointCount++;
            }
        }
    }

    sigma = std::sqrt(sigma/pointCount);

    pointCount = 0;
    for(int v0 = 0; v0 < corresps.rows; v0++)
    {
        for(int u0 = 0; u0 < corresps.cols; u0++)
        {
            if(corresps.at<int>(v0,u0) != -1)
            {
                int u1, v1;
                get2shorts(corresps.at<int>(v0,u0), u1, v1);

                Point3f p0 = levelCloud0.at<Point3f>(v0, u0), tp0; // tp0 - transformed p0
                // recomputing the values here is even faster than saving them in previos cycle
                tp0.x = p0.x * Rt_ptr[0] + p0.y * Rt_ptr[1] + p0.z * Rt_ptr[2] + Rt_ptr[3];
                tp0.y = p0.x * Rt_ptr[4] + p0.y * Rt_ptr[5] + p0.z * Rt_ptr[6] + Rt_ptr[7];
                tp0.z = p0.x * Rt_ptr[8] + p0.y * Rt_ptr[9] + p0.z * Rt_ptr[10] + Rt_ptr[11];

                Vec3f n1 = levelNormals1.at<Vec3f>(v1, u1);
                Point3f v = levelCloud1.at<Point3f>(v1,u1) - tp0;
                double diff = n1[0] * v.x + n1[1] * v.y + n1[2] * v.z;

                double w = sigma + std::abs(diff);
                w = w > DBL_EPSILON ? 1./w : 1.;

                computeC((double*)C.ptr(pointCount), tp0, n1 * w);

                dV_dt.at<double>(pointCount) = w * diff;

                pointCount++;
            }
        }
    }

    A = C;
    B = dV_dt;
}

static
void randomSubsetOfMask(Mat& mask, float part)
{
    const int minPointsCount = 1000; // minimum point count (we can process them fast)
    const int nonzeros = countNonZero(mask);
    const int needCount = std::max(minPointsCount, int(mask.total() * part));
    if(needCount < nonzeros)
    {
        RNG rng;
        Mat subset(mask.size(), CV_8UC1, Scalar(0));

        int subsetSize = 0;
        while(subsetSize < needCount)
        {
            int y = rng(mask.rows);
            int x = rng(mask.cols);
            if(mask.at<uchar>(y,x))
            {
                subset.at<uchar>(y,x) = 255;
                mask.at<uchar>(y,x) = 0;
                subsetSize++;
            }
        }
        mask = subset;
    }
}

bool cv::RGBDICPOdometry(cv::Mat& Rt, const Mat& initRt,
                         const cv::Mat& image0, const cv::Mat& _depth0, const cv::Mat& validMask0,
                         const cv::Mat& image1, const cv::Mat& _depth1, const cv::Mat& validMask1,
                         const cv::Mat& cameraMatrix,
                         std::vector<cv::Ptr<cv::RgbdNormals> >& normalComputers,
                         float minDepth, float maxDepth, float maxDepthDiff,
                         const std::vector<int>& iterCounts, 
                         const std::vector<float>& minGradientMagnitudes, float icpPointsPart,
                         int methodType)
{
    const int sobelSize = 3;
    const double sobelScale = 1./8;

    Mat depth0 = _depth0.clone(),
        depth1 = _depth1.clone();

    // check RGB-D input data
    CV_Assert(!image0.empty());
    CV_Assert(image0.type() == CV_8UC1);
    CV_Assert(depth0.type() == CV_32FC1 && depth0.size() == image0.size());

    CV_Assert(image1.size() == image0.size());
    CV_Assert(image1.type() == CV_8UC1);
    CV_Assert(depth1.type() == CV_32FC1 && depth1.size() == image0.size());

    // check masks
    CV_Assert(validMask0.empty() || (validMask0.type() == CV_8UC1 && validMask0.size() == image0.size()));
    CV_Assert(validMask1.empty() || (validMask1.type() == CV_8UC1 && validMask1.size() == image0.size()));

    // check camera params
    CV_Assert(cameraMatrix.type() == CV_32FC1 && cameraMatrix.size() == Size(3,3));

    // other checks
    CV_Assert(iterCounts.empty() || minGradientMagnitudes.empty() ||
               minGradientMagnitudes.size() == iterCounts.size());
    CV_Assert(initRt.empty() || (initRt.type()==CV_64FC1 && initRt.size()==Size(4,4)));

    vector<int> defaultIterCounts;
    vector<float> defaultMinGradMagnitudes;
    vector<int> const* iterCountsPtr = &iterCounts;
    vector<float> const* minGradientMagnitudesPtr = &minGradientMagnitudes;

    if(iterCounts.empty() || minGradientMagnitudes.empty())
    {
        defaultIterCounts.resize(4);
        defaultIterCounts[0] = 7;
        defaultIterCounts[1] = 7;
        defaultIterCounts[2] = 7;
        defaultIterCounts[3] = 10;

        defaultMinGradMagnitudes.resize(4);
        defaultMinGradMagnitudes[0] = 8;
        defaultMinGradMagnitudes[1] = 4;
        defaultMinGradMagnitudes[2] = 2;
        defaultMinGradMagnitudes[3] = 1;

        iterCountsPtr = &defaultIterCounts;
        minGradientMagnitudesPtr = &defaultMinGradMagnitudes;
    }

    preprocessDepth(depth0, depth1, validMask0, validMask1, minDepth, maxDepth);

    vector<Mat> pyramidImage0, pyramidDepth0, pyramidCloud0,
                pyramidImage1, pyramidDepth1, pyramidCloud1,
                pyramid_dI_dx1, pyramid_dI_dy1,
                pyramidNormals1, pyramidTexturedMask1, pyramidNormalMask1,
                pyramidCameraMatrix;

    buildPyramids(image0, image1, depth0, depth1,
                   cameraMatrix, sobelSize, sobelScale, *minGradientMagnitudesPtr,
                   pyramidImage0, pyramidDepth0, pyramidImage1, pyramidDepth1,
                   pyramid_dI_dx1, pyramid_dI_dy1,
                   pyramidCloud0, pyramidCloud1,
                   pyramidNormals1,
                   pyramidTexturedMask1, pyramidNormalMask1, pyramidCameraMatrix,
                   normalComputers);

    Mat resultRt = initRt.empty() ? Mat::eye(4,4,CV_64FC1) : initRt.clone();
    Mat currRt, ksi;
    for(int level = (int)iterCountsPtr->size() - 1; level >= 0; level--)
    {
        const Mat& levelCameraMatrix = pyramidCameraMatrix[level];

        const Mat& levelImage0 = pyramidImage0[level];
        const Mat& levelDepth0 = pyramidDepth0[level];
        const Mat& levelCloud0 = pyramidCloud0[level];
        const Mat& levelImage1 = pyramidImage1[level];
        const Mat& levelDepth1 = pyramidDepth1[level];
        const Mat& levelCloud1 = pyramidCloud1[level];
        const Mat& level_dI_dx1 = pyramid_dI_dx1[level];
        const Mat& level_dI_dy1 = pyramid_dI_dy1[level];

        CV_Assert(level_dI_dx1.type() == CV_16S);
        CV_Assert(level_dI_dy1.type() == CV_16S);

        const double fx = levelCameraMatrix.at<double>(0,0);
        const double fy = levelCameraMatrix.at<double>(1,1);
        const double determinantThreshold = 1e-6;

        Mat corresps1(levelImage0.size(), CV_32SC1);
        Mat corresps2(levelDepth1.size(), CV_32SC1);

        // Run transformation search on current level iteratively.
        Mat mask1, mask2;
        if(methodType & RGBD_ODOMETRY)
            mask1 = pyramidTexturedMask1[level];
        if(methodType & ICP_ODOMETRY)
        {
            mask2 = pyramidNormalMask1[level];
            randomSubsetOfMask(mask2, icpPointsPart);
        }

        for(int iter = 0; iter < (*iterCountsPtr)[level]; iter ++)
        {
            int correspsCount1 = 0, correspsCount2 = 0;

            if(methodType & RGBD_ODOMETRY)
            {
                correspsCount1 = computeCorresp(levelCameraMatrix, levelCameraMatrix.inv(), resultRt.inv(DECOMP_SVD),
                                                levelDepth0, levelDepth1, mask1, maxDepthDiff,
                                                corresps1);
            }
            if(methodType & ICP_ODOMETRY)
            {
                correspsCount2 = computeCorresp(levelCameraMatrix, levelCameraMatrix.inv(), resultRt.inv(DECOMP_SVD),
                                                 levelDepth0, levelDepth1, mask2, maxDepthDiff,
                                                 corresps2);
            }

            if(correspsCount1 < 6 && correspsCount2 < 6)
                break;

            Mat A1, B1, A2, B2;
            Mat A(6, 6, CV_64FC1, Scalar(0)), B(6, 1, CV_64FC1, Scalar(0));
            if(correspsCount1 >= 6)
            {
                computeAB(levelImage0, levelCloud0,
                          levelImage1, level_dI_dx1, level_dI_dy1,
                          corresps1, correspsCount1,
                          fx, fy, sobelScale, A1, B1);

                B1 = A1.t() * B1;
                A1 = A1.t() * A1;

                A += A1;
                B += B1;
            }

            if(correspsCount2 >= 6)
            {
                computeAB(levelCloud0, resultRt,
                          levelCloud1, pyramidNormals1[level],
                          corresps2, correspsCount2,
                          A2, B2);

                B2 = A2.t() * B2;
                A2 = A2.t() * A2;

                A += A2;
                B += B2;
            }

            bool solutionExist = solveSystem(A, B, determinantThreshold, ksi);

            if(!solutionExist)
                break;

            computeProjectiveMatrix(ksi, currRt);

            resultRt = currRt * resultRt;
        }
    }
    Rt = resultRt;

    return !Rt.empty();
}
