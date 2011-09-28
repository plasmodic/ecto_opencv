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

#include <limits>

#include <ecto/ecto.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "impl/depth_to_3d.h"

using ecto::tendrils;
namespace calib
{
  struct DepthTo3dSparse
  {
    typedef std::vector<cv::Point2f> points_t;
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Mat>("K", "The calibration matrix").required(true);
      inputs.declare<cv::Mat>("points", "The 2d coordinates (matrix with 2 channels)").required(true);
      inputs.declare<cv::Mat>("depth", "The depth image").required(true);
      outputs.declare<cv::Mat>("points3d", "The 3d points, same dimensions as the input, 3 channels (x, y and z).");
    }

    /** Get the 2d keypoints and figure out their 3D position from the depth map
     * @param inputs
     * @param outputs
     * @return
     */
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      cv::Mat K;
      inputs["K"] >> K;
      const cv::Mat &depth = inputs.get<cv::Mat>("depth"), &uv = inputs.get<cv::Mat>("points");

      std::vector<cv::Mat> uv_vec(2);
      cv::split(uv, uv_vec);
      cv::Mat points3d;
      depthTo3dSparse(K, depth, uv_vec[0], uv_vec[1], points3d);

      outputs["points3d"] << points3d;

      return ecto::OK;
    }
  };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  struct DepthTo3d
  {
    /**
     * @param K
     * @param depth the depth image
     * @param mask the mask of the points to consider (can be empty)
     * @param points3d the resulting 3d points
     */
    static void
    depthTo3d(const cv::Mat& K, const cv::Mat& depth, const cv::Mat& mask, cv::Mat& points3d)
    {
      // Create 3D points in one go.
      if (!mask.empty())
        depthTo3dMask(K, depth, mask, points3d);
      else
        calib::depthTo3d(K, depth, points3d);
    }

    typedef std::vector<cv::Point2f> points_t;

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Mat>("K", "The calibration matrix").required(true);
      inputs.declare<cv::Mat>("depth", "The depth image").required(true);
      inputs.declare<cv::Mat>("mask", "The mask of the points to send back");
      outputs.declare<cv::Mat>(
          "points3d", "The 3d points, height by width (or 1 by n_points if mask) with 3 channels (x, y and z)");
    }

    /** Get the 2d keypoints and figure out their 3D position from the depth map
     * @param inputs
     * @param outputs
     * @return
     */
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      cv::Mat K, depth, mask;
      inputs["K"] >> K;
      inputs["depth"] >> depth;
      inputs["mask"] >> mask;

      cv::Mat points3d;
      depthTo3d(K, depth, mask, points3d);

      outputs["points3d"] << points3d;
      return 0;
    }
  };
}
using namespace calib;
ECTO_CELL(calib, DepthTo3d, "DepthTo3d", "Converts depth to 3d points.")
ECTO_CELL(calib, DepthTo3dSparse, "DepthTo3dSparse", "Converts depth to 3d points.")
