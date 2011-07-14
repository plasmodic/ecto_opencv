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

namespace projector
{
void
depth23d(const cv::Mat& K, const cv::Mat& depth, cv::Mat& points3d, const cv::Point2f& topleft)
{
  cv::Mat_<float> scaled_points = cv::Mat_<float>(3, depth.size().area());
  // Create the scaled keypoints
  cv::Size depth_size = depth.size();
  cv::Mat_<float>::const_iterator begin = depth.begin<float> (), end = depth.end<float> ();
  cv::Mat_<float>::iterator sp_begin = scaled_points.begin();
  cv::Point2f point = topleft;
  while (begin != end)
  {
    float d = *(begin++);
    *(sp_begin++) = point.x * d;
    *(sp_begin++) = point.y * d;
    *(sp_begin++) = d;
    if (point.x < depth_size.width + topleft.x)
    {
      point.x += 1;
    }
    else
    {
      point.y += 1;
      point.x = topleft.x;
    }
  }
  // Figure out the 3D points
  cv::Mat_<float> final_points_tmp;
  cv::solve(K, scaled_points, final_points_tmp);
  points3d = final_points_tmp;
}

using ecto::tendrils;

/** Ecto implementation of a module that takes
 *
 */
struct PlaneFitter
{
  typedef std::vector<cv::Point2f> points_t;
  static void declare_params(tendrils& p)
  {
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat>("K", "The calibration matrix");
    inputs.declare<cv::Mat>("depth", "The depth image");
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
  }

  /** Get the 2d keypoints and figure out their 3D position from the depth map
   * @param inputs
   * @param outputs
   * @return
   */
  int process(tendrils& inputs, tendrils& outputs)
  {
    cv::Mat K,depth;
    inputs["K"] >> K;
    inputs["depth"] >> depth;
    K.clone().convertTo(K, CV_32F);
    cv::Mat points3d;
    int roi_size = 10;
    cv::Rect roi(depth.size().width/2 - roi_size/2,depth.size().height/2 - roi_size/2,roi_size,roi_size);
    cv::Mat depth_sub = depth(roi);
    depth23d(K,depth_sub,points3d,roi.tl());
    std::cout << points3d <<std::endl;
    return 0;
  }
};
}
ECTO_CELL(projector, projector::PlaneFitter, "PlaneFitter", "Finds the plane.");
