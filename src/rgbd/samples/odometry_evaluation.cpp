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
    
    const float fx = 517.3f,
                fy = 516.5f,
                cx = 318.6f,
                cy = 255.3f;
    Mat cameraMatrix = Mat::eye(3,3,CV_32FC1);
    {
        cameraMatrix.at<float>(0,0) = fx;
        cameraMatrix.at<float>(1,1) = fy;
        cameraMatrix.at<float>(0,2) = cx;
        cameraMatrix.at<float>(1,2) = cy;
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

    Ptr<Odometry> odometry = Algorithm::create<Odometry>("RGBD." + string(argv[3]) + "Odometry");
    if(odometry.empty())
    {
        cout << "Can not create Odometry algorithm. Check the passed odometry name." << endl;
        return -1;
    }
    odometry->set("cameraMatrix", cameraMatrix);
    
    for(int i = 0; !file.eof(); i++)
    {
        string str;
        std::getline(file, str);
        if(str.empty()) break;
        if(str.at(0) == '#') continue; /* comment */

        Mat image, depth;
        // Read one pair (rgb and depth)
        // example: 1305031453.359684 rgb/1305031453.359684.png 1305031453.374112 depth/1305031453.374112.png
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
            depth = depth_flt;
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
                tm.stop();

                CV_Assert(res);
                cout << "Time " << tm.getTimeSec() << endl;
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
}
