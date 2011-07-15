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

#include <boost/foreach.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>

#include "common.h"

using ecto::tendrils;

/** Ecto implementation of a module that takes
 *
 */
struct DepthWarper
{
  typedef std::vector<cv::Point2f> points_t;
  static void declare_params(tendrils& p)
  {
    p.declare<std::string>("projection_file");
    p.declare<int>("width");
    p.declare<int>("height");
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat>("depth", "The depth image");
    inputs.declare<cv::Mat>("K", "The Kinect calibration matrix");
    outputs.declare<cv::Mat>("output", "The depth image");
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    readOpenCVCalibration(P_, params.get<std::string>("projection_file"));
  }

  /** Get the 2d keypoints and figure out their 3D position from the depth map
   * @param inputs
   * @param outputs
   * @return
   */
  int process(tendrils& inputs, tendrils& outputs)
  {
    cv::Mat depth, K;
    inputs.get<cv::Mat>("depth").convertTo(depth, CV_32F);
    inputs.get<cv::Mat>("K").convertTo(K, CV_32F);

    int width = 640, height = 480;
    cv::Mat_<float> x = cv::Mat_<float>(1, width), y = cv::Mat_<float>(height, 1);
    for (unsigned int i = 0; i < width; ++i)
      x(0, i) = i;
    for (unsigned int i = 0; i < height; ++i)
      y(i, 0) = i;
    x = cv::repeat(x, height, 1);
    y = cv::repeat(y, 1, width);

    // Create the scaled keypoints
    x = x.mul(depth);
    y = y.mul(depth);

    std::vector<cv::Mat> channels;
    channels.push_back(x);
    channels.push_back(y);
    channels.push_back(depth);

    cv::Mat scaled_image;
    cv::merge(channels, scaled_image);
    cv::Mat_<float> scaled_points = scaled_image.reshape(1, width * height).t();

    // Figure out the 3D points
    cv::Mat_<float> points_3d;
    cv::solve(K, scaled_points, points_3d);

    points_3d.resize(4, cv::Scalar(1));
    cv::Mat_<float> points_2d_tmp = P_ * points_3d;

    // Give a color depending on the depth
    cv::Mat_<float> points_2d(2, width * height);
    cv::Mat row = points_2d.row(0);
    cv::Mat((points_2d_tmp.row(0) / points_2d_tmp.row(2))).copyTo(row);
    row = points_2d.row(1);
    cv::Mat((points_2d_tmp.row(1) / points_2d_tmp.row(2))).copyTo(row);

    cv::Mat drawn_image = cv::Mat::zeros(height, width, CV_8UC3);
    for (int n = 0; n < points_3d.cols; ++n)
    {
      if (points_3d(2, n) == points_3d(2, n))
      {
        int j = points_2d(1, n);
        int i = points_2d(0, n);
        int kernel_size = 5;
        for (int jj = std::max(j - kernel_size, 0); jj < std::min(j + kernel_size, height); ++jj)
          for (int ii = std::max(i - kernel_size, 0); ii < std::min(i + kernel_size, width); ++ii)
            drawn_image.at<cv::Vec3b>(jj, ii) = cv::Vec3b(255.0 / 1 * (points_3d(2, n)-0.5), 255, 255);
      }
    }
    cv::Mat & output = outputs.get<cv::Mat>("output");
    output = drawn_image.clone();
    cv::cvtColor(drawn_image, output, CV_HSV2RGB);

    return 0;
  }
private:
  cv::Mat P_;
};

ECTO_CELL(projector, DepthWarper, "DepthWarper", "Warps an image based on the depth.");
