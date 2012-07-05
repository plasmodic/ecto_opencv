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

using namespace cv;

enum
{
    RGBD_ODOMETRY = 1, 
    ICP_ODOMETRY = 2, 
    MERGED_ODOMETRY = RGBD_ODOMETRY + ICP_ODOMETRY
};

static
void setDefaultIterCounts(Mat& iterCounts)
{
    iterCounts.create(1,4,CV_32SC1);
    iterCounts.at<int>(0) = 7;
    iterCounts.at<int>(1) = 7;
    iterCounts.at<int>(2) = 7;
    iterCounts.at<int>(3) = 10;
}

static
void setDefaultMinGradientMagnitudes(Mat& minGradientMagnitudes)
{
    minGradientMagnitudes.create(1,4,CV_32FC1);
    minGradientMagnitudes.at<float>(0) = 8.f;
    minGradientMagnitudes.at<float>(1) = 4.f;
    minGradientMagnitudes.at<float>(2) = 2.f;
    minGradientMagnitudes.at<float>(3) = 1.f;
}

static
bool RGBDICPOdometryImpl(cv::Mat& Rt, const Mat& initRt,
                         const cv::Mat& image0, const cv::Mat& _depth0, const cv::Mat& mask0,
                         const cv::Mat& image1, const cv::Mat& _depth1, const cv::Mat& mask1,
                         const cv::Mat& cameraMatrix,
                         float minDepth, float maxDepth, float maxDepthDiff,
                         const std::vector<int>& iterCounts, 
                         const std::vector<float>& minGradientMagnitudes, float icpPointsPart,
                         std::vector<cv::Ptr<cv::RgbdNormals> >& normalComputers,
                         int method);

static 
void checkFramesImpl(const Mat& image0, const Mat& depth0, const Mat& mask0,
                     const Mat& image1, const Mat& depth1, const Mat& mask1)
{
    CV_Assert(!image0.empty());
    CV_Assert(image0.type() == CV_8UC1);
    CV_Assert(depth0.type() == CV_32FC1 && depth0.size() == image0.size());
    CV_Assert(mask0.empty() || (mask0.type() == CV_8UC1 && mask0.size() == image0.size()));

    CV_Assert(image1.size() == image0.size());
    CV_Assert(image1.type() == CV_8UC1);
    CV_Assert(depth1.type() == CV_32FC1 && depth1.size() == image0.size());
    CV_Assert(mask1.empty() || (mask1.type() == CV_8UC1 && mask1.size() == image0.size()));
}

static 
void checkFramesImpl(const Mat& depth0, const Mat& mask0,
                     const Mat& depth1, const Mat& mask1)
{
    CV_Assert(!depth0.empty());
    CV_Assert(depth0.type() == CV_32FC1);
    CV_Assert(mask0.empty() || (mask0.type() == CV_8UC1 && mask0.size() == depth0.size()));

    CV_Assert(depth1.size() == depth0.size());
    CV_Assert(depth1.type() == CV_32FC1);
    CV_Assert(mask1.empty() || (mask1.type() == CV_8UC1 && mask1.size() == depth1.size()));
}

namespace cv
{
bool Odometry::compute(const Mat& image0, const Mat& depth0, const Mat& mask0,
                       const Mat& image1, const Mat& depth1, const Mat& mask1, 
                       Mat& Rt, const Mat& initRt) const
{
    checkParams(); // they can be changed
    checkFrames(image0, depth0, mask0, 
                image1, depth1, mask1);
    CV_Assert(initRt.empty() || (initRt.type() == CV_64FC1 && initRt.size() == Size(4,4)));
    
    return computeImpl(image0, depth0, mask0,
                       image1, depth1, mask1,
                       initRt, Rt);
}

RgbdOdometry::RgbdOdometry() :
    minDepth(DEFAULT_MIN_DEPTH()), maxDepth(DEFAULT_MAX_DEPTH()), maxDepthDiff(DEFAULT_MAX_DEPTH_DIFF())
{
    setDefaultIterCounts(iterCounts);
    setDefaultMinGradientMagnitudes(minGradientMagnitudes);
}

RgbdOdometry::RgbdOdometry(const Mat& _cameraMatrix, 
                           float _minDepth, float _maxDepth, float _maxDepthDiff,
                           const vector<int>& _iterCounts,
                           const vector<float>& _minGradientMagnitudes) :
                           cameraMatrix(_cameraMatrix),
                           minDepth(_minDepth), maxDepth(_maxDepth), maxDepthDiff(_maxDepthDiff),
                           iterCounts(_iterCounts), 
                           minGradientMagnitudes(_minGradientMagnitudes)
{
    if(iterCounts.empty() || minGradientMagnitudes.empty())
    {
        setDefaultIterCounts(iterCounts);
        setDefaultMinGradientMagnitudes(minGradientMagnitudes);
    }
}

void RgbdOdometry::checkParams() const
{
    CV_Assert(cameraMatrix.size() == Size(3,3) && cameraMatrix.type() == CV_32FC1);
    CV_Assert(minGradientMagnitudes.size() == iterCounts.size());
}

void RgbdOdometry::checkFrames(const Mat& image0, const Mat& depth0, const Mat& mask0,
                               const Mat& image1, const Mat& depth1, const Mat& mask1) const
{
    checkFramesImpl(image0, depth0, mask0,
                    image1, depth1, mask1);
}

bool RgbdOdometry::computeImpl(const Mat& image0, const Mat& depth0, const Mat& mask0,
                               const Mat& image1, const Mat& depth1, const Mat& mask1, 
                               const Mat& initRt, Mat& Rt) const
{
    vector<cv::Ptr<cv::RgbdNormals> > nc;
    return RGBDICPOdometryImpl(Rt, initRt, image0, depth0, mask0, image1, depth1, mask1,
                               cameraMatrix, minDepth, maxDepth, maxDepthDiff,
                               iterCounts, minGradientMagnitudes, 0.f,
                               nc, RGBD_ODOMETRY);
}

ICPOdometry::ICPOdometry() :
    minDepth(DEFAULT_MIN_DEPTH()), maxDepth(DEFAULT_MAX_DEPTH()), 
    maxDepthDiff(DEFAULT_MAX_DEPTH_DIFF()), pointsPart(DEFAULT_USED_POINTS_PART())
{
    setDefaultIterCounts(iterCounts);
}
    
ICPOdometry::ICPOdometry(const Mat& _cameraMatrix, 
                         float _minDepth, float _maxDepth, float _maxDepthDiff,
                         float _pointsPart, const vector<int>& _iterCounts) :
                         cameraMatrix(_cameraMatrix), 
                         minDepth(_minDepth), maxDepth(_maxDepth), maxDepthDiff(_maxDepthDiff),
                         pointsPart(_pointsPart), iterCounts(_iterCounts)
{
    if(iterCounts.empty())
        setDefaultIterCounts(iterCounts);
}
                 
void ICPOdometry::checkParams() const
{
    CV_Assert(pointsPart > 0. && pointsPart <= 1.);
    CV_Assert(cameraMatrix.size() == Size(3,3) && cameraMatrix.type() == CV_32FC1);
}

void ICPOdometry::checkFrames(const Mat& /*image0*/, const Mat& depth0, const Mat& mask0,
                              const Mat& /*image1*/, const Mat& depth1, const Mat& mask1) const
{
    checkFramesImpl(depth0, mask0, depth1, mask1);
}

bool ICPOdometry::computeImpl(const Mat& /*image0*/, const Mat& depth0, const Mat& mask0,
                              const Mat& /*image1*/, const Mat& depth1, const Mat& mask1,
                              const Mat& initRt, Mat& Rt) const
{
    return RGBDICPOdometryImpl(Rt, initRt, Mat(), depth0, mask0, Mat(), depth1, mask1,
                               cameraMatrix, minDepth, maxDepth, maxDepthDiff,
                               iterCounts, vector<float>(), pointsPart,
                               normalComputers, ICP_ODOMETRY);
}

RgbdICPOdometry::RgbdICPOdometry() :
    minDepth(DEFAULT_MIN_DEPTH()), maxDepth(DEFAULT_MAX_DEPTH()), 
    maxDepthDiff(DEFAULT_MAX_DEPTH_DIFF()), pointsPart(DEFAULT_USED_POINTS_PART())
{
    setDefaultIterCounts(iterCounts);
    setDefaultMinGradientMagnitudes(minGradientMagnitudes);
}

RgbdICPOdometry::RgbdICPOdometry(const Mat& _cameraMatrix, 
                                 float _minDepth, float _maxDepth, float _maxDepthDiff,
                                 float _pointsPart, const vector<int>& _iterCounts,
                                 const vector<float>& _minGradientMagnitudes) :
                                 cameraMatrix(_cameraMatrix),
                                 minDepth(_minDepth), maxDepth(_maxDepth), maxDepthDiff(_maxDepthDiff),
                                 pointsPart(_pointsPart), iterCounts(_iterCounts),
                                 minGradientMagnitudes(_minGradientMagnitudes)
{
    if(iterCounts.empty() || minGradientMagnitudes.empty())
    {
        setDefaultIterCounts(iterCounts);
        setDefaultMinGradientMagnitudes(minGradientMagnitudes);
    }
}

void RgbdICPOdometry::checkParams() const
{
    CV_Assert(pointsPart > 0. && pointsPart <= 1.);
    CV_Assert(cameraMatrix.size() == Size(3,3) && cameraMatrix.type() == CV_32FC1);
    CV_Assert(minGradientMagnitudes.size() == iterCounts.size());
}

void RgbdICPOdometry::checkFrames(const Mat& image0, const Mat& depth0, const Mat& mask0,
                                  const Mat& image1, const Mat& depth1, const Mat& mask1) const
{
    checkFramesImpl(image0, depth0, mask0,
                    image1, depth1, mask1);
}

bool RgbdICPOdometry::computeImpl(const Mat& image0, const Mat& depth0, const Mat& mask0,
                                  const Mat& image1, const Mat& depth1, const Mat& mask1,
                                  const Mat& initRt, Mat& Rt) const
{
    return RGBDICPOdometryImpl(Rt, initRt, image0, depth0, mask0, image1, depth1, mask1,
                               cameraMatrix, minDepth, maxDepth, maxDepthDiff,
                               iterCounts, minGradientMagnitudes, pointsPart,
                               normalComputers, MERGED_ODOMETRY);
};
} // namespace cv

///////////////////////////////////////////////////////////////////////////////////////

static
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
    // TODO: check computeProjectiveMatrix when there is not eigen library, 
    //       because it gives less accurate pose of the camera
    Rt = Mat::eye(4, 4, CV_64FC1);

    Mat R = Rt(Rect(0,0,3,3));
    Mat rvec = ksi.rowRange(0,3);

    Rodrigues(rvec, R);

    Rt.at<double>(0,3) = ksi.at<double>(3);
    Rt.at<double>(1,3) = ksi.at<double>(4);
    Rt.at<double>(2,3) = ksi.at<double>(5);
#endif
}

static
void preprocessDepth(Mat& depth, const Mat& mask, float minDepth, float maxDepth)
{
    CV_DbgAssert(depth.type() == CV_32FC1);
    if(mask.empty())
    {
        for(int y = 0; y < depth.rows; y++)
        {
            float *depth_row = depth.ptr<float>(y);
            for(int x = 0; x < depth.cols; x++)
            {
                float& d = depth_row[x];
                if(/*!cvIsNaN(d) && */(d > maxDepth || d < minDepth || d <= 0))
                    d = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
    else
    {
        for(int y = 0; y < depth.rows; y++)
        {
            float *depth_row = depth.ptr<float>(y);
            const uchar *mask_row = mask.ptr<uchar>(y);
            for(int x = 0; x < depth.cols; x++)
            {
                float& d = depth_row[x];
                if(!mask_row[x] || (/*!cvIsNaN(d) && */(d > maxDepth || d < minDepth || d <= 0)))
                    d = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
}

static
void buildCameraMatrixPyramid(const Mat& cameraMatrix, int levels, vector<Mat>& pyramidCameraMatrix)
{
    pyramidCameraMatrix.resize(levels);
    
    Mat cameraMatrix_dbl;
    cameraMatrix.convertTo(cameraMatrix_dbl, CV_64FC1);
    
    for(int i = 0; i < levels; i++)
    {
        Mat levelCameraMatrix = i == 0 ? cameraMatrix_dbl : 0.5f * pyramidCameraMatrix[i-1];
        levelCameraMatrix.at<double>(2,2) = 1.;
        pyramidCameraMatrix[i] = levelCameraMatrix;
    }
}

static
void buildGradientPyramids(const vector<Mat>& pyramidImage, 
                           const vector<float>& minGradMagnitudes,
                           int sobelSize, double sobelScale,
                           vector<Mat>& pyramid_dI_dx, 
                           vector<Mat>& pyramid_dI_dy,
                           vector<Mat>& pyramidTexturedMask)
{
    const float sobelScale2_inv = 1.f / (sobelScale * sobelScale);
    
    pyramid_dI_dx.resize(pyramidImage.size());
    pyramid_dI_dy.resize(pyramidImage.size());
    pyramidTexturedMask.resize(pyramidImage.size());

    for(size_t i = 0; i < pyramidTexturedMask.size(); i++)
    {
        const float minScaledGradMagnitude2 = minGradMagnitudes[i] * minGradMagnitudes[i] * sobelScale2_inv;
        
        Sobel(pyramidImage[i], pyramid_dI_dx[i], CV_16S, 1, 0, sobelSize);
        Sobel(pyramidImage[i], pyramid_dI_dy[i], CV_16S, 0, 1, sobelSize);

        const Mat& dIdx = pyramid_dI_dx[i];
        const Mat& dIdy = pyramid_dI_dy[i];

        Mat texturedMask(dIdx.size(), CV_8UC1, Scalar(0));
        
        for(int y = 0; y < dIdx.rows; y++)
        {
            const short *dIdx_row = dIdx.ptr<short>(y);
            const short *dIdy_row = dIdy.ptr<short>(y);
            uchar *texturedMask_row = texturedMask.ptr<uchar>(y);
            for(int x = 0; x < dIdx.cols; x++)
            {
                float magnitude2 = static_cast<float>(dIdx_row[x] * dIdx_row[x] + dIdy_row[x] * dIdy_row[x]);
                if(magnitude2 >= minScaledGradMagnitude2)
                    texturedMask_row[x] = 255;
            }
        }
        pyramidTexturedMask[i] = texturedMask;
    }
}

static
void buildNormalPyramids(const vector<Mat>& pyramidDepth0, const vector<Mat>& pyramidDepth1, 
                         const vector<Mat>& pyramidCameraMatrix,
                         vector<Mat>& pyramidVertices0, vector<Mat>& pyramidVertices1,
                         vector<Mat>& pyramidNormals1, vector<Mat>& pyramidNormalMask1,
                         vector<Ptr<RgbdNormals> >& normalComputers)
{
    size_t pyramidLevels = pyramidCameraMatrix.size();
    pyramidVertices0.resize(pyramidLevels);
    pyramidVertices1.resize(pyramidLevels);
    pyramidNormals1.resize(pyramidLevels);
    pyramidNormalMask1.resize(pyramidLevels);

    if(normalComputers.empty() || normalComputers.size() != pyramidLevels)
        normalComputers.resize(pyramidLevels);

    for(size_t i = 0; i < pyramidLevels; i++)
    {
        depthTo3d(pyramidDepth0[i], pyramidCameraMatrix[i], pyramidVertices0[i]);
        depthTo3d(pyramidDepth1[i], pyramidCameraMatrix[i], pyramidVertices1[i]);

        const Mat& levelDepth1 = pyramidDepth1[i];
        const Mat& levelVertices1 = pyramidVertices1[i];
        const Mat& K = pyramidCameraMatrix[i];
        
        if(normalComputers[i].empty() ||
           normalComputers[i]->get<int>("rows") != levelDepth1.rows ||
           normalComputers[i]->get<int>("cols") != levelDepth1.cols ||
           cv::norm(normalComputers[i]->get<Mat>("K"), K) > FLT_EPSILON)
        {
            normalComputers[i] = new RgbdNormals(levelDepth1.rows, levelDepth1.cols, 
                                                 levelDepth1.depth(), K, 3);
        }
            
        Mat& normals1 = pyramidNormals1[i];
        normals1 = (*normalComputers[i])(pyramidVertices1[i]);
        
        CV_Assert(normals1.type() == CV_32FC3);
        Mat normalMask1 = Mat(levelVertices1.size(), CV_8UC1, Scalar(0));
        for(int y = 0; y < normals1.rows; y++)
        {
            const Vec3f *normals1_row = normals1.ptr<Vec3f>(y);
            uchar *normalMask1_row = normalMask1.ptr<uchar>(y);
            for(int x = 0; x < normals1.cols; x++)
            {
                Vec3f n = normals1_row[x];
                if(!cvIsNaN(n[0]))
                {
                    CV_DbgAssert(!cvIsNaN(n[1]) && !cvIsNaN(n[2]));
                    normalMask1_row[x] = 255;
                }
            }
        }
        pyramidNormalMask1[i] = normalMask1;
    }
}


static
void buildPyramids(const Mat& image0, const Mat& image1,
                   const Mat& depth0, const Mat& depth1,
                   const Mat& cameraMatrix, int sobelSize, double sobelScale, int pyramidLevels,
                   const vector<float>& minGradientMagnitudes,
                   vector<Mat>& pyramidImage0, vector<Mat>& pyramidDepth0,
                   vector<Mat>& pyramidImage1, vector<Mat>& pyramidDepth1,
                   vector<Mat>& pyramid_dI_dx1, vector<Mat>& pyramid_dI_dy1,
                   vector<Mat>& pyramidCloud0, vector<Mat>& pyramidCloud1,
                   vector<Mat>& pyramidNormals1,
                   vector<Mat>& pyramidTexturedMask1, vector<Mat>& pyramidNormalMask1,
                   vector<Mat>& pyramidCameraMatrix,
                   vector<Ptr<RgbdNormals> >& normalComputers,
                   int method)
{
    buildCameraMatrixPyramid(cameraMatrix, pyramidLevels, pyramidCameraMatrix);
    
    buildPyramid(depth0, pyramidDepth0, pyramidLevels - 1);
    buildPyramid(depth1, pyramidDepth1, pyramidLevels - 1);
    
    if(method & RGBD_ODOMETRY)
    {
        buildPyramid(image0, pyramidImage0, pyramidLevels - 1);
        buildPyramid(image1, pyramidImage1, pyramidLevels - 1);

        buildGradientPyramids(pyramidImage1, minGradientMagnitudes, sobelSize, sobelScale, 
                              pyramid_dI_dx1, pyramid_dI_dy1, pyramidTexturedMask1);
    }
                              
    if(method == RGBD_ODOMETRY)
    {
        pyramidCloud0.resize(pyramidImage1.size());
        for(size_t i = 0; i < pyramidImage1.size(); i++)
            depthTo3d(pyramidDepth0[i], pyramidCameraMatrix[i], pyramidCloud0[i]);
    }
    
    if(method & ICP_ODOMETRY)
        buildNormalPyramids(pyramidDepth0, pyramidDepth1, pyramidCameraMatrix,
                            pyramidCloud0, pyramidCloud1,
                            pyramidNormals1, pyramidNormalMask1,
                            normalComputers);
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
int computeCorresps(const Mat& K, const Mat& K_inv, const Mat& Rt,
                    const Mat& depth0, const Mat& depth1, const Mat& mask1, float maxDepthDiff,
                    Mat& corresps)
{
    CV_Assert(K.type() == CV_64FC1);
    CV_Assert(K_inv.type() == CV_64FC1);
    CV_Assert(Rt.type() == CV_64FC1);

    corresps.create(depth1.size(), CV_32SC1);
    corresps.setTo(-1);
    
    Rect r(0, 0, depth1.cols, depth1.rows);
    Mat Kt = Rt(Rect(3,0,1,3)).clone();
    Kt = K * Kt;
    const double * Kt_ptr = reinterpret_cast<const double *>(Kt.ptr());

    AutoBuffer<float> buf(3 * (depth1.cols + depth1.rows));
    float *KRK_inv0_u1 = buf;
    float *KRK_inv1_v1_plus_KRK_inv2 = KRK_inv0_u1 + depth1.cols;
    float *KRK_inv3_u1 = KRK_inv1_v1_plus_KRK_inv2 + depth1.rows;
    float *KRK_inv4_v1_plus_KRK_inv5 = KRK_inv3_u1 + depth1.cols;
    float *KRK_inv6_u1 = KRK_inv4_v1_plus_KRK_inv5 + depth1.rows;
    float *KRK_inv7_v1_plus_KRK_inv8 = KRK_inv6_u1 + depth1.cols;
    {
        Mat R = Rt(Rect(0,0,3,3)).clone();

        Mat KRK_inv = K * R * K_inv;
        const double * KRK_inv_ptr = reinterpret_cast<const double *>(KRK_inv.ptr());
        for(int u1 = 0; u1 < depth1.cols; u1++)
        {
            KRK_inv0_u1[u1] = KRK_inv_ptr[0] * u1;
            KRK_inv3_u1[u1] = KRK_inv_ptr[3] * u1;
            KRK_inv6_u1[u1] = KRK_inv_ptr[6] * u1;
        }
        
        for(int v1 = 0; v1 < depth1.rows; v1++)
        {
            KRK_inv1_v1_plus_KRK_inv2[v1] = KRK_inv_ptr[1] * v1 + KRK_inv_ptr[2];
            KRK_inv4_v1_plus_KRK_inv5[v1] = KRK_inv_ptr[4] * v1 + KRK_inv_ptr[5];
            KRK_inv7_v1_plus_KRK_inv8[v1] = KRK_inv_ptr[7] * v1 + KRK_inv_ptr[8];
        }
    }

    int correspCount = 0;
    for(int v1 = 0; v1 < depth1.rows; v1++)
    {
        const float *depth1_row = depth1.ptr<float>(v1);
        const uchar *mask1_row = mask1.ptr<uchar>(v1);
        for(int u1 = 0; u1 < depth1.cols; u1++)
        {
            float d1 = depth1_row[u1];
            if(mask1_row[u1] && !cvIsNaN(d1))
            {
                float transformed_d1 = static_cast<float>(d1 * (KRK_inv6_u1[u1] + KRK_inv7_v1_plus_KRK_inv8[v1]) + Kt_ptr[2]);
                if(transformed_d1 > 0)
                {
                    float transformed_d1_inv = 1.f / transformed_d1;
                    int u0 = cvRound(transformed_d1_inv * (d1 * (KRK_inv0_u1[u1] + KRK_inv1_v1_plus_KRK_inv2[v1]) + Kt_ptr[0]));
                    int v0 = cvRound(transformed_d1_inv * (d1 * (KRK_inv3_u1[u1] + KRK_inv4_v1_plus_KRK_inv5[v1]) + Kt_ptr[1]));
                    
                    float d0 = depth0.at<float>(v0,u0);
                    if(!cvIsNaN(d0) && std::abs(transformed_d1 - d0) <= maxDepthDiff && 
                        r.contains(Point(u0,v0)))
                    {
                        int c = corresps.at<int>(v0,u0);
                        if(c != -1)
                        {
                            int exist_u1, exist_v1;
                            get2shorts(c, exist_u1, exist_v1);

                            float exist_d1 = (float)(depth1.at<float>(exist_v1,exist_u1) * 
                                (KRK_inv6_u1[exist_u1] + KRK_inv7_v1_plus_KRK_inv8[exist_v1]) + Kt_ptr[2]);

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

static inline
void calcRgbdEquationCoeffs(double* C, double dIdx, double dIdy, const Point3f& p3d, double fx, double fy)
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
void calcICPEquationCoeffs(double* C, const Point3f& p0, const Vec3f& n1)
{
    C[0] = -p0.z * n1[1] + p0.y * n1[2];
    C[1] =  p0.z * n1[0] - p0.x * n1[2];
    C[2] = -p0.y * n1[0] + p0.x * n1[1];
    C[3] = n1[0];
    C[4] = n1[1];
    C[5] = n1[2];
}

static 
void calcRgbdLsmMatrices(const Mat& image0, const Mat& cloud0,
               const Mat& image1, const Mat& dI_dx1, const Mat& dI_dy1,
               const Mat& corresps, int correspsCount,
               double fx, double fy, double sobelScale,
               Mat& A, Mat& B)
{
    A.create(correspsCount, 6, CV_64FC1);
    B.create(correspsCount, 1, CV_64FC1);

    double sigma = 0;
    int pointCount = 0;
    AutoBuffer<float> diffs(correspsCount);
    float * diffs_ptr = diffs;
    for(int v0 = 0; v0 < corresps.rows; v0++)
    {
        const int* corresps_row = corresps.ptr<int>(v0);
        const uchar* image0_row = image0.ptr<uchar>(v0);
        for(int u0 = 0; u0 < corresps.cols; u0++)
        {
            if(corresps_row[u0] != -1)
            {
                int u1, v1;
                get2shorts(corresps_row[u0], u1, v1);

                diffs_ptr[pointCount] = static_cast<float>(static_cast<int>(image0_row[u0]) - static_cast<int>(image1.at<uchar>(v1,u1)));
                sigma += diffs_ptr[pointCount] * diffs_ptr[pointCount];
                pointCount++;
            }
        }
    }
    sigma = std::sqrt(sigma/pointCount);

    pointCount = 0;
    for(int v0 = 0; v0 < corresps.rows; v0++)
    {
        const int* corresps_row = corresps.ptr<int>(v0);
        const Point3f* cloud0_row = cloud0.ptr<Point3f>(v0);
        for(int u0 = 0; u0 < corresps.cols; u0++)
        {
            if(corresps_row[u0] != -1)
            {
                int u1, v1;
                get2shorts(corresps_row[u0], u1, v1);

                double w = sigma + std::abs(diffs_ptr[pointCount]);
                w = w > DBL_EPSILON ? 1./w : 1.;

                double w_sobelScale = w * sobelScale;
                calcRgbdEquationCoeffs(A.ptr<double>(pointCount),
                         w_sobelScale * dI_dx1.at<short int>(v1,u1),
                         w_sobelScale * dI_dy1.at<short int>(v1,u1),
                         cloud0_row[u0], fx, fy);

                B.at<double>(pointCount) = w * diffs_ptr[pointCount];
                pointCount++;
            }
        }
    }
}

static
void calcICPLsmMatrices(const Mat& levelCloud0, const Mat& Rt,
               const Mat& levelCloud1, const Mat& levelNormals1,
               const Mat& corresps, int correspsCount,
               Mat& A, Mat& B)
{
    A.create(correspsCount, 6, CV_64FC1);
    B.create(correspsCount, 1, CV_64FC1);

    CV_Assert(Rt.type() == CV_64FC1);
    const double * Rt_ptr = reinterpret_cast<const double*>(Rt.data);

    double sigma = 0;
    int pointCount = 0;
    AutoBuffer<float> diffs(correspsCount);
    float * diffs_ptr = diffs;
    AutoBuffer<Point3f> transformedPoints0(correspsCount);
    Point3f * tps0_ptr = transformedPoints0;
    for(int v0 = 0; v0 < corresps.rows; v0++)
    {
        const int* corresps_row = corresps.ptr<int>(v0);
        const Point3f* levelCloud0_row = levelCloud0.ptr<Point3f>(v0);
        for(int u0 = 0; u0 < corresps.cols; u0++)
        {
            if(corresps_row[u0] != -1)
            {
                int u1, v1;
                get2shorts(corresps_row[u0], u1, v1);

                const Point3f& p0 = levelCloud0_row[u0];
                Point3f tp0; 
                tp0.x = p0.x * Rt_ptr[0] + p0.y * Rt_ptr[1] + p0.z * Rt_ptr[2] + Rt_ptr[3];
                tp0.y = p0.x * Rt_ptr[4] + p0.y * Rt_ptr[5] + p0.z * Rt_ptr[6] + Rt_ptr[7];
                tp0.z = p0.x * Rt_ptr[8] + p0.y * Rt_ptr[9] + p0.z * Rt_ptr[10] + Rt_ptr[11];

                Vec3f n1 = levelNormals1.at<Vec3f>(v1, u1);
                Point3f v = levelCloud1.at<Point3f>(v1,u1) - tp0;
                
                tps0_ptr[pointCount] = tp0;
                diffs_ptr[pointCount] = n1[0] * v.x + n1[1] * v.y + n1[2] * v.z;
                sigma += diffs_ptr[pointCount] * diffs_ptr[pointCount];
                
                pointCount++;
            }
        }
    }

    sigma = std::sqrt(sigma/pointCount);

    pointCount = 0;
    for(int v0 = 0; v0 < corresps.rows; v0++)
    {
        const int* corresps_row = corresps.ptr<int>(v0);
        for(int u0 = 0; u0 < corresps.cols; u0++)
        {
            if(corresps_row[u0] != -1)
            {
                int u1, v1;
                get2shorts(corresps_row[u0], u1, v1);

                double w = sigma + std::abs(diffs_ptr[pointCount]);
                w = w > DBL_EPSILON ? 1./w : 1.;

                calcICPEquationCoeffs(A.ptr<double>(pointCount), tps0_ptr[pointCount], levelNormals1.at<Vec3f>(v1, u1) * w);
                B.at<double>(pointCount) = w * diffs_ptr[pointCount];

                pointCount++;
            }
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

static
bool RGBDICPOdometryImpl(cv::Mat& Rt, const Mat& initRt,
                     const cv::Mat& image0, const cv::Mat& _depth0, const cv::Mat& mask0,
                     const cv::Mat& image1, const cv::Mat& _depth1, const cv::Mat& mask1,
                     const cv::Mat& cameraMatrix,
                     float minDepth, float maxDepth, float maxDepthDiff,
                     const std::vector<int>& iterCounts, 
                     const std::vector<float>& minGradientMagnitudes, float icpPointsPart,
                     std::vector<cv::Ptr<cv::RgbdNormals> >& normalComputers,
                     int method)
{
    const int sobelSize = 3;
    const double sobelScale = 1./8.;
    Mat depth0 = _depth0.clone(),
        depth1 = _depth1.clone();

    preprocessDepth(depth0, mask0, minDepth, maxDepth);
    preprocessDepth(depth1, mask1, minDepth, maxDepth);

    vector<Mat> pyramidImage0, pyramidDepth0, pyramidCloud0,
                pyramidImage1, pyramidDepth1, pyramidCloud1,
                pyramid_dI_dx1, pyramid_dI_dy1,
                pyramidNormals1, pyramidTexturedMask1, pyramidNormalMask1,
                pyramidCameraMatrix;

    buildPyramids(image0, image1, depth0, depth1, cameraMatrix, 
                  sobelSize, sobelScale, iterCounts.size(),
                  minGradientMagnitudes,
                  pyramidImage0, pyramidDepth0, pyramidImage1, pyramidDepth1,
                  pyramid_dI_dx1, pyramid_dI_dy1, pyramidCloud0, pyramidCloud1,
                  pyramidNormals1,
                  pyramidTexturedMask1, pyramidNormalMask1, pyramidCameraMatrix,
                  normalComputers, method);

    Mat resultRt = initRt.empty() ? Mat::eye(4,4,CV_64FC1) : initRt.clone();
    Mat currRt, ksi;
    for(int level = iterCounts.size() - 1; level >= 0; level--)
    {
        const Mat& levelCameraMatrix = pyramidCameraMatrix[level];
        const Mat& levelCameraMatrix_inv = levelCameraMatrix.inv(DECOMP_SVD);

        const Mat& levelDepth0 = pyramidDepth0[level];  
        const Mat& levelDepth1 = pyramidDepth1[level];

        const double fx = levelCameraMatrix.at<double>(0,0);
        const double fy = levelCameraMatrix.at<double>(1,1);
        const double determinantThreshold = 1e-6;

        // 1 - rgbd; 2 - icp
        Mat A1, B1, A2, B2; 
        Mat corresps1, corresps2;
        Mat mask1, mask2;
        
        if(method & RGBD_ODOMETRY)
        {
            corresps1.create(levelDepth0.size(), CV_32SC1);
            mask1 = pyramidTexturedMask1[level];
        }
        if(method & ICP_ODOMETRY) 
        {
            corresps2.create(levelDepth0.size(), CV_32SC1);
            mask2 = pyramidNormalMask1[level];
            randomSubsetOfMask(mask2, icpPointsPart);
        }

        // Run transformation search on current level iteratively.
        for(int iter = 0; iter < iterCounts[level]; iter ++)
        {
            int correspsCount1 = 0, correspsCount2 = 0;
            Mat resultRt_inv = resultRt.inv(DECOMP_SVD);
            
            if(method & RGBD_ODOMETRY)
                correspsCount1 = computeCorresps(levelCameraMatrix, levelCameraMatrix_inv, resultRt_inv,
                                                 levelDepth0, levelDepth1, mask1, maxDepthDiff,
                                                 corresps1);
            if(method & ICP_ODOMETRY)
                correspsCount2 = computeCorresps(levelCameraMatrix, levelCameraMatrix_inv, resultRt_inv,
                                                levelDepth0, levelDepth1, mask2, maxDepthDiff,
                                                corresps2);

            if(correspsCount1 < 6 && correspsCount2 < 6)
                break;

            Mat AtA(6, 6, CV_64FC1, Scalar(0)), AtB(6, 1, CV_64FC1, Scalar(0));
            if(correspsCount1 >= 6)
            {
                calcRgbdLsmMatrices(pyramidImage0[level], pyramidCloud0[level],
                                    pyramidImage1[level], pyramid_dI_dx1[level], pyramid_dI_dy1[level],
                                    corresps1, correspsCount1, fx, fy, sobelScale, A1, B1);
                AtA += A1.t() * A1;
                AtB += A1.t() * B1;
            }
            if(correspsCount2 >= 6)
            {
                calcICPLsmMatrices(pyramidCloud0[level], resultRt,
                                   pyramidCloud1[level], pyramidNormals1[level],
                                   corresps2, correspsCount2, A2, B2);
                AtA += A2.t() * A2;
                AtB += A2.t() * B2;
            }

            bool solutionExist = solveSystem(AtA, AtB, determinantThreshold, ksi);
            if(!solutionExist)
                break;

            computeProjectiveMatrix(ksi, currRt);
            resultRt = currRt * resultRt;
        }
    }
    Rt = resultRt;

    return !Rt.empty();
}
