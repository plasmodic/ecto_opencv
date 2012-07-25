/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#include <opencv2/rgbd/rgbd.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <dirent.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

#define BILATERAL_FILTER 0 // if 1 then bilateral filter will be used for the depth
#if BILATERAL_FILTER
#define SEQUENTIAL_MERGE_METHOD 0 // if 1 then the passed type of Odometry will be ignored and sequence ICP + RGBD (pure rotation) will be used.
#endif


#if BILATERAL_FILTER
// This function is almost the full copy-paste of bilateralFilter_32f from imgproc/src/smooth.cpp of OpenCV
// but it ignores invalid depth pixel! The original OpenCV filter does some bad work with invalid depth values.
static void
depthBilateralFilter_32f(const Mat& src, Mat& dst, int d,
                         double sigma_color, double sigma_space,
                         int borderType=BORDER_DEFAULT )
{
    int cn = src.channels();
    int i, j, k, maxk, radius;
    double minValSrc=-1, maxValSrc=1;
    const int kExpNumBinsPerChannel = 1 << 12;
    int kExpNumBins = 0;
    float lastExpVal = 1.f;
    float len, scale_index;
    Size size = src.size();

    CV_Assert( (src.type() == CV_32FC1 || src.type() == CV_32FC3) &&
        src.type() == dst.type() && src.size() == dst.size() &&
        src.data != dst.data );

    if( sigma_color <= 0 )
        sigma_color = 1;
    if( sigma_space <= 0 )
        sigma_space = 1;

    double gauss_color_coeff = -0.5/(sigma_color*sigma_color);
    double gauss_space_coeff = -0.5/(sigma_space*sigma_space);

    if( d <= 0 )
        radius = cvRound(sigma_space*1.5);
    else
        radius = d/2;
    radius = MAX(radius, 1);
    d = radius*2 + 1;

    // compute the min/max range for the input image (even if multichannel)
    minMaxLoc( src.reshape(1), &minValSrc, &maxValSrc );

    // temporary copy of the image with borders for easy processing
    Mat temp;
    copyMakeBorder( src, temp, radius, radius, radius, radius, borderType );
    patchNaNs(temp);

    TickMeter tm;
    tm.start();
    // allocate lookup tables
    vector<float> _space_weight(d*d);
    vector<int> _space_ofs(d*d);
    float* space_weight = &_space_weight[0];
    int* space_ofs = &_space_ofs[0];

    // assign a length which is slightly more than needed
    len = (float)(maxValSrc - minValSrc) * cn;
    kExpNumBins = kExpNumBinsPerChannel * cn;
    vector<float> _expLUT(kExpNumBins+2);
    float* expLUT = &_expLUT[0];

    scale_index = kExpNumBins/len;

    // initialize the exp LUT
    for( i = 0; i < kExpNumBins+2; i++ )
    {
        if( lastExpVal > 0.f )
        {
            double val =  i / scale_index;
            expLUT[i] = (float)std::exp(val * val * gauss_color_coeff);
            lastExpVal = expLUT[i];
        }
        else
            expLUT[i] = 0.f;
    }

    // initialize space-related bilateral filter coefficients
    for( i = -radius, maxk = 0; i <= radius; i++ )
        for( j = -radius; j <= radius; j++ )
        {
            double r = std::sqrt((double)i*i + (double)j*j);
            if( r > radius )
                continue;
            space_weight[maxk] = (float)std::exp(r*r*gauss_space_coeff);
            space_ofs[maxk++] = (int)(i*(temp.step/sizeof(float)) + j*cn);
        }

    tm.stop();
    std::cout << "Exp time " << tm.getTimeSec() << std::endl;

    CV_Assert(maxk % 2 == 1);
    for( i = 0; i < size.height; i++ )
    {
        const float* sptr = (const float*)(temp.data + (i+radius)*temp.step) + radius*cn;
        float* dptr = (float*)(dst.data + i*dst.step);

        for( j = 0; j < size.width; j++ )
        {
            float sum = 0, wsum = 0;
            float val0 = sptr[j];
            if(val0 == 0.f)
                continue;

            for( k = 0; k < maxk; k++ )
            {
                float val = sptr[j + space_ofs[k]];
                float alpha = (float)(std::abs(val - val0));
                if(val == 0.f)
                    continue;

                alpha *= scale_index;
                int idx = cvFloor(alpha);
                alpha -= idx;
                float w = space_weight[k]*(expLUT[idx] + alpha*(expLUT[idx+1] - expLUT[idx]));
                sum += val*w;
                wsum += w;
            }
            dptr[j] = (float)(sum/wsum);
        }
    }
}
#endif

static
void writeResults( const string& filename, const vector<string>& timestamps, const vector<Mat>& Rt )
{
    CV_Assert( timestamps.size() == Rt.size() );

    ofstream file( filename.c_str() );
    if( !file.is_open() )
        return;

    cout.precision(4);
    for( size_t i = 0; i < Rt.size(); i++ )
    {
        const Mat& Rt_curr = Rt[i];
        if( Rt_curr.empty() )
            continue;

        CV_Assert( Rt_curr.type() == CV_64FC1 );

        Mat R = Rt_curr(Rect(0,0,3,3)), rvec;
        Rodrigues(R, rvec);
        double alpha = norm( rvec );
        if(alpha > DBL_MIN)
            rvec = rvec / alpha;

        double cos_alpha2 = std::cos(0.5 * alpha);
        double sin_alpha2 = std::sin(0.5 * alpha);

        rvec *= sin_alpha2;

        CV_Assert( rvec.type() == CV_64FC1 );
        // timestamp tx ty tz qx qy qz qw
        file << timestamps[i] << " " << fixed
             << Rt_curr.at<double>(0,3) << " " << Rt_curr.at<double>(1,3) << " " << Rt_curr.at<double>(2,3) << " "
             << rvec.at<double>(0) << " " << rvec.at<double>(1) << " " << rvec.at<double>(2) << " " << cos_alpha2 << endl;

    }
    file.close();
}

static
void setCameraMatrixFreiburg1(float& fx, float& fy, float& cx, float& cy)
{
    fx = 517.3f; fy = 516.5f; cx = 318.6f; cy = 255.3f;
}

static
void setCameraMatrixFreiburg2(float& fx, float& fy, float& cx, float& cy)
{
    fx = 520.9f; fy = 521.0f; cx = 325.1f; cy = 249.7f;
}

/*
 * This sample helps to evaluate odometry on TUM datasets and benchmark http://vision.in.tum.de/data/datasets/rgbd-dataset.
 * At this link you can find instructions for evaluation. The sample runs some opencv odometry and saves a camera trajectory
 * to file of format that the benchmark requires. Saved file can be used for online evaluation.
 */
int main(int argc, char** argv)
{
    if(argc != 4)
    {
        cout << "Format: file_with_rgb_depth_pairs trajectory_file odometry_name [Rgbd or ICP or RgbdICP]" << endl;
        return -1;
    }
    
    vector<string> timestamps;
    vector<Mat> Rts;

    const string filename = argv[1];
    ifstream file( filename.c_str() );
    if( !file.is_open() )
        return -1;

    char dlmrt = '/';
    size_t pos = filename.rfind(dlmrt);
    string dirname = pos == string::npos ? "" : filename.substr(0, pos) + dlmrt;

    const int timestampLength = 17;
    const int rgbPathLehgth = 17+8;
    const int depthPathLehgth = 17+10;

    Mat image_prev, gray_prev, image_curr, gray_curr;
    Mat depth_prev, depth_curr;
    
    float fx = 525.0f, // default
          fy = 525.0f,
          cx = 319.5f,
          cy = 239.5f;
    if(filename.find("freiburg1") != string::npos)
        setCameraMatrixFreiburg1(fx, fy, cx, cy);
    if(filename.find("freiburg2") != string::npos)
        setCameraMatrixFreiburg2(fx, fy, cx, cy);
    Mat cameraMatrix = Mat::eye(3,3,CV_32FC1);
    {
        cameraMatrix.at<float>(0,0) = fx;
        cameraMatrix.at<float>(1,1) = fy;
        cameraMatrix.at<float>(0,2) = cx;
        cameraMatrix.at<float>(1,2) = cy;
    }

    string odometryName = string(argv[3]);
#if SEQUENTIAL_MERGE_METHOD
    odometryName = "ICP";
#endif

    Ptr<Odometry> odometry = Algorithm::create<Odometry>("RGBD." + string(argv[3]) + "Odometry");
    if(odometry.empty())
    {
        cout << "Can not create Odometry algorithm. Check the passed odometry name." << endl;
        return -1;
    }
    odometry->set("cameraMatrix", cameraMatrix);

#if SEQUENTIAL_MERGE_METHOD
    RgbdOdometry rgbd(cameraMatrix);
    rgbd.set("transformType", Odometry::ROTATION);
#endif

    for(int i = 0; !file.eof(); i++)
    {
        string str;
        std::getline(file, str);
        if(str.empty()) break;
        if(str.at(0) == '#') continue; /* comment */

        Mat image, depth;
        // Read one pair (rgb and depth)
        // example: 1305031453.359684 rgb/1305031453.359684.png 1305031453.374112 depth/1305031453.374112.png
#if BILATERAL_FILTER
        TickMeter tm_bilateral_filter;
#endif
        {
            string rgbFilename = str.substr(timestampLength + 1, rgbPathLehgth );
            string timestap = str.substr(0, timestampLength);
            string depthFilename = str.substr(2*timestampLength + rgbPathLehgth + 3, depthPathLehgth );

            image = imread(dirname + rgbFilename);
            depth = imread(dirname + depthFilename, -1);

            CV_Assert(!image.empty());
            CV_Assert(!depth.empty());
            CV_Assert(depth.type() == CV_16UC1);

            cout << i << " " << rgbFilename << " " << depthFilename << endl;

            // scale depth
            Mat depth_flt;
            depth.convertTo(depth_flt, CV_32FC1, 1.f/5000.f);
#if not BILATERAL_FILTER
            depth = depth_flt;
#else
            tm_bilateral_filter.start();
            depth = Mat(depth_flt.size(), CV_32FC1, Scalar(0));
            const double depth_sigma = 0.1; // in meters; it was OK even with 0.3
            const double space_sigma = 4.5;  // in pixels
            depthBilateralFilter_32f(depth_flt, depth, -1, depth_sigma, space_sigma);
            tm_bilateral_filter.stop();
            cout << "Time filter " << tm_bilateral_filter.getTimeSec() << endl;
#endif
            timestamps.push_back( timestap );
        }

        {
            image_curr = image.clone();
            cvtColor(image_curr, gray_curr, CV_BGR2GRAY);
            depth_curr = depth.clone();
            
            Mat Rt = Mat::eye(4,4,CV_64FC1);
            if(!image_prev.empty())
            {
                TickMeter tm;
                tm.start();
                bool res = odometry->compute(gray_curr, depth_curr, Mat(), 
                                             gray_prev, depth_prev, Mat(),
                                             Rt);
#if SEQUENTIAL_MERGE_METHOD
                rgbd.compute(gray_curr, depth_curr, Mat(),
                             gray_prev, depth_prev, Mat(),
                             Rt, Rt.clone());
#endif
                tm.stop();

                CV_Assert(res);
                cout << "Time " << tm.getTimeSec() << endl;
#if BILATERAL_FILTER
                cout << "Time ratio " << tm_bilateral_filter.getTimeSec() / tm.getTimeSec() << endl;
#endif
            }

            if( Rts.empty() )
                Rts.push_back( Rt );
            else
            {
                Mat& prevRt = *Rts.rbegin();
                cout << "Rt " << Rt << endl;
                Rts.push_back( prevRt * Rt );
            }

            std::swap(image_prev, image_curr);
            std::swap(gray_prev, gray_curr);
            std::swap(depth_prev, depth_curr);
        }
    }

    writeResults(argv[2], timestamps, Rts);

    return 0;
}
