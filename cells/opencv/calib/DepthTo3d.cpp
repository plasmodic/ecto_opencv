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

#if CV_MAJOR_VERSION == 2
#include <opencv2/rgbd/rgbd.hpp>
using namespace cv;
#else
#include <opencv2/rgbd.hpp>
using namespace cv::rgbd;
#endif

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

      cv::Mat points3d;
      depthTo3dSparse(depth, K, uv, points3d);

      outputs["points3d"] << points3d;

      return ecto::OK;
    }
  };

  struct Select3d
  {
    typedef std::vector<cv::Point2f> points_t;
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      typedef Select3d C;
      inputs.declare(&C::points, "points", "The 2d coordinates (matrix with 2 channels)").required(true);
      inputs.declare(&C::points3d_in, "points3d", "The 3d points to select from.").required(true);
      outputs.declare(&C::points3d_out, "points3d",
                      "The 3d points, same dimensions as the input, 3 channels (x, y and z).");
    }

    /** Get the 2d keypoints and figure out their 3D position from the depth map
     * @param inputs
     * @param outputs
     * @return
     */
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      // Make sure we use float types
      cv::Mat_<float> uv_float;
      if (points->depth() == (CV_32F))
        uv_float = *points;
      else
        points->convertTo(uv_float, CV_32F);

      // Make sure we use float types
      cv::Mat_<cv::Point3f> points3d, output;
      if (points3d_in->depth() == (CV_32F))
        points3d = *points3d_in;
      else
        points3d_in->convertTo(points3d, CV_32F);

      output.reserve(uv_float.rows);
      for (int i = 0, end = uv_float.rows; i < end; i++)
      {
        cv::Point2f p;
        p.x = uv_float(i, 0);
        p.y = uv_float(i, 1);
        cv::Point3f p3d = points3d(int(p.y+0.5f), int(p.x+0.5f));
        output.push_back(p3d);
      }
      *points3d_out = output;
      return ecto::OK;
    }
    ecto::spore<cv::Mat> points, points3d_in, points3d_out;
  };

  struct Select3dRegion
  {
    typedef Select3dRegion C;
    typedef std::vector<cv::Point2f> points_t;
    static void
    declare_params(tendrils& params)
    {
      params.declare(&C::radius, "radius", "A radius, in pixel with which to select a plane, from the center.");
    }
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&C::image, "image", "To base the size off of.").required(true);
      inputs.declare(&C::points3d_in, "points3d", "The 3d points to select from.").required(true);
      outputs.declare(&C::points3d_out, "points3d",
                      "The 3d points, same dimensions as the input, 3 channels (x, y and z).");
    }

    inline bool
    is_nan_safe(const cv::Point3f& p) const
    {
      return p.x != p.x || p.y != p.y || p.z != p.z;
    }

    /** Get the 2d keypoints and figure out their 3D position from the depth map
     * @param inputs
     * @param outputs
     * @return
     */
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      float rad = 50;
      if (radius.user_supplied())
      {
        rad = *radius;
      }
      // Make sure we use float types
      cv::Mat_<cv::Point3f> points3d, output;
      if (points3d_in->depth() == (CV_32F))
        points3d = *points3d_in;
      else
        points3d_in->convertTo(points3d, CV_32F);

      for (int v = 0, v_end = image->rows; v != v_end; ++v)
      {
        for (int u = 0, u_end = image->cols; u != u_end; ++u)
        {
          cv::Vec2f p(u - u_end / 2.0, v - v_end / 2.0);
          if (cv::norm(p) > rad)
            continue;
          cv::Point3f p3d = points3d(v, u);
          if (!is_nan_safe(p3d))
            output.push_back(p3d);
        }
      }
      *points3d_out = output;
      return ecto::OK;
    }
    ecto::spore<cv::Mat> image, points3d_in, points3d_out;
    ecto::spore<float> radius, r_width, r_height;
  };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  struct DepthTo3d
  {
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
      depthTo3d(depth, K, points3d, mask);

      outputs["points3d"] << points3d;
      return 0;
    }
  };
}
using namespace calib;
ECTO_CELL(calib, DepthTo3d, "DepthTo3d", "Converts depth to 3d points.")
ECTO_CELL(calib, DepthTo3dSparse, "DepthTo3dSparse", "Converts depth to 3d points.")
ECTO_CELL(calib, Select3d, "Select3d", "Select 3D points given 2D locations.")
ECTO_CELL(calib, Select3dRegion, "Select3dRegion", "Select 3D points given a radius in the center of the image.")
