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
struct FiducialWarper
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
    inputs.declare<cv::Mat>("R", "The original 2D pattern");
    inputs.declare<cv::Mat>("T", "The points we want to 3d-fy (an aternative to the keypoints)");
    inputs.declare<bool>("found", "The calibration matrix");
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
    bool found;
    inputs["found"] >> found;

    if (!found)
      return 0;

    cv::Mat R, T;
    inputs.get<cv::Mat>("R").convertTo(R, CV_32F);
    inputs.get<cv::Mat>("T").convertTo(T, CV_32F);

    cv::Point3f x(0.25, 0, 0);
    cv::Point3f y(0.25, 0.25, 0);
    cv::Point3f z(0, 0.25, 0);
    cv::Point3f o(0, 0, 0);
    std::vector<cv::Point3f> op(4);
    op[1] = x, op[2] = y, op[3] = z, op[0] = o;
    cv::Mat_<float> points_3d = cv::Mat(op).reshape(1).t();
    T * cv::Mat_<float>::ones(1, 4);
    cv::Mat_<float> points_kinect = R * points_3d + T * cv::Mat_<float>::ones(1, 4);
    points_kinect.resize(4, cv::Scalar(1));
    cv::Mat points_homogeneous = P_ * points_kinect;
    cv::Mat_<float> points_2d(4, 2);
    cv::convertPointsFromHomogeneous(points_homogeneous.t(), points_2d);
    points_2d.push_back((const cv::Mat &)(points_2d.row(0)));

    cv::Mat drawn_image = cv::Mat::zeros(480, 640, CV_8UC3);

    for (unsigned int i = 0; i < 4; ++i)
      cv::line(drawn_image, cv::Point2f(points_2d(i, 0), points_2d(i, 1)),
               cv::Point2f(points_2d(i + 1, 0), points_2d(i + 1, 1)), cv::Scalar(255, 255, 255), 10);

    outputs.get<cv::Mat>("output") = drawn_image;

    return 0;
  }
private:
  cv::Mat P_;
};

ECTO_CELL(projector, FiducialWarper, "FiducialWarper", "Figures out the calibration of the projector.");
