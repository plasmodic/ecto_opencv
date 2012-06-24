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

#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/rgbd/rgbd.hpp>
#include <string>
#include <numeric>

using ecto::tendrils;

using namespace cv;
using namespace std;

template<class ImageElemType>
static void
warpImage(const cv::Mat& image, const Mat& depth, const Mat& Rt, const Mat& cameraMatrix, const Mat& distCoeff,
          Mat& warpedImage)
{
  const Rect rect = Rect(0, 0, image.cols, image.rows);

  vector<Point2f> points2d;
  Mat cloud, transformedCloud;

  depthTo3d(depth, cameraMatrix, cloud);
  perspectiveTransform(cloud, transformedCloud, Rt);
  projectPoints(transformedCloud.reshape(3, 1), Mat::eye(3, 3, CV_64FC1), Mat::zeros(3, 1, CV_64FC1), cameraMatrix,
                distCoeff, points2d);

  Mat pointsPositions(points2d);
  pointsPositions = pointsPositions.reshape(2, image.rows);

  warpedImage.create(image.size(), image.type());
  warpedImage = Scalar::all(0);

  Mat zBuffer(image.size(), CV_32FC1, FLT_MAX);

  for (int y = 0; y < image.rows; y++)
  {
    for (int x = 0; x < image.cols; x++)
    {
      const Point3f p3d = transformedCloud.at<Point3f>(y, x);
      const Point2i p2d = pointsPositions.at<Point2f>(y, x);
      if (!cvIsNaN(cloud.at<Point3f>(y, x).z) && cloud.at<Point3f>(y, x).z > 0 && rect.contains(p2d)
          && zBuffer.at<float>(p2d) > p3d.z)
      {
        warpedImage.at<ImageElemType>(p2d) = image.at<ImageElemType>(y, x);
        zBuffer.at<float>(p2d) = p3d.z;
      }
    }
  }
}

namespace rgbd
{
  struct Odometry
  {
    static void
    declare_params(tendrils& params)
    {
      /*params.declare<double>("angle_thresh", "The angle thresh hold.", CV_PI / 36);
       params.declare<bool>("reset", "Reset observations.", false);
       params.declare<unsigned>("n_desired", "The number of desired views", std::numeric_limits<unsigned>::max());*/
    }
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&Odometry::current_image_, "image", "The current visual frame.").required(true);
      inputs.declare(&Odometry::current_depth_, "depth", "The current depth frame.").required(true);
      inputs.declare(&Odometry::K_, "K", "The camera intrinsic parameter matrix.").required(true);

      outputs.declare(&Odometry::R_, "R", "The rotation of the camera pose with respect to the previous frame.");
      outputs.declare(&Odometry::T_, "T", "The rotation of the camera pose with respect to the previous frame.");
      outputs.declare(&Odometry::warp_, "image", "The warped previous image.");
    }
    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      // Convert the current image to grayscale
      cv::Mat current_image_gray, current_image;
      if (current_image_->channels() == 3)
      {
        cv::cvtColor(*current_image_, current_image_gray, CV_BGR2GRAY);
        current_image_->copyTo(current_image);
      }
      else
      {
        current_image_->copyTo(current_image_gray);
        cv::merge(std::vector<cv::Mat>(3, *current_image_), current_image);
      }

      // Change the depth to meters
      cv::Mat current_depth_meters;
      rescaleDepth(*current_depth_, CV_32FC1, current_depth_meters);

      // Odometry is only possible when we have a previous frame
      static int counter = 0;
      ++counter;
      if ((previous_image_gray_.empty()) || (counter == 30))
      {
        first_image_ = current_image;
        first_depth_meters_ = current_depth_meters;
        previous_pose_ = cv::Mat::eye(4, 4, CV_32F);
        previous_image_ = current_image;
        previous_image_gray_ = current_image_gray;
        previous_depth_meters_ = current_depth_meters;
        return ecto::OK;
      }

      cv::TickMeter tm;
      cv::Mat Rt;

      cv::Mat cameraMatrix;
      K_->convertTo(cameraMatrix, CV_32FC1);

      std::vector<int> iterCounts(4);
      iterCounts[0] = 7;
      iterCounts[1] = 7;
      iterCounts[2] = 7;
      iterCounts[3] = 10;

      std::vector<float> minGradMagnitudes(4);
      minGradMagnitudes[0] = 12;
      minGradMagnitudes[1] = 5;
      minGradMagnitudes[2] = 3;
      minGradMagnitudes[3] = 1;

      const float minDepth = 0.f; //in meters
      const float maxDepth = 3.f; //in meters
      const float maxDepthDiff = 0.07f; //in meters

      tm.start();

      std::vector<cv::Ptr<cv::RgbdNormals> > normalComputers;
      float icpPointsPart = 0;

      /*bool isFound = cv::RGBDOdometry(Rt, cv::Mat(), previous_image_gray_, previous_depth_meters_, cv::Mat(),
       current_image_gray, current_depth_meters, cv::Mat(), cameraMatrix, minDepth,
       maxDepth, maxDepthDiff, iterCounts, minGradMagnitudes, cv::RIGID_BODY_MOTION);*/

      bool isFound = cv::RGBDICPOdometry(Rt, cv::Mat(), previous_image_gray_, previous_depth_meters_, cv::Mat(),
                                         current_image_gray, current_depth_meters, cv::Mat(), cameraMatrix,
                                         normalComputers, minDepth, maxDepth, maxDepthDiff, iterCounts,
                                         minGradMagnitudes, icpPointsPart, cv::RGBD_ODOMETRY);

      if (isFound)
        previous_pose_ = cv::Mat_<float>(Rt) * previous_pose_;

      tm.stop();

      std::cout << "Rt = " << Rt << std::endl;
      std::cout << "Time = " << tm.getTimeSec() << " sec." << std::endl;

      if (!isFound)
      {
        std::cout << "Rigid body motion cann't be estimated for given RGBD data." << std::endl;
        return -1;
      }

      // Just for display
      if (isFound)
      {
        cv::Mat warpedImage0;
        const Mat distCoeff(1, 5, CV_32FC1, Scalar(0));

        std::cout << previous_pose_ << std::endl;
        warpImage<Point3_<uchar> >(first_image_, first_depth_meters_, previous_pose_, cameraMatrix, distCoeff,
                                   warpedImage0);
        warpedImage0.copyTo(*warp_);
      }

      // Keep track of the frames
      previous_image_ = current_image;
      previous_image_gray_ = current_image_gray;
      previous_depth_meters_ = current_depth_meters;

      return ecto::OK;
    }

    ecto::spore<cv::Mat> K_;
    ecto::spore<cv::Mat> current_image_;
    ecto::spore<cv::Mat> current_depth_;
    cv::Mat first_image_;
    cv::Mat first_depth_meters_;
    cv::Mat previous_image_gray_;
    cv::Mat previous_image_;
    cv::Mat previous_depth_meters_;
    cv::Mat_<float> previous_pose_;
    ecto::spore<cv::Mat> warp_;

    /** The output rotation matrix */
    ecto::spore<cv::Mat> R_;
    /** The output translation matrix */
    ecto::spore<cv::Mat> T_;
  };
}

ECTO_CELL(rgbd, rgbd::Odometry, "Odometry", "Uses the RGBDOdometry to figure out where the camera is.")
