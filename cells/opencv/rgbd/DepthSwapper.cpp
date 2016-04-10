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

#include <ecto/ecto.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#if CV_MAJOR_VERSION == 3
#include <opencv2/rgbd.hpp>
#else
#include <opencv2/rgbd/rgbd.hpp>
#endif

using ecto::tendrils;
namespace rgbd
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  struct DepthSwapper
  {
    static void
    declare_params(tendrils & params)
    {
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&DepthSwapper::depth_, "depth", "The depth map").required(true);
      inputs.declare(&DepthSwapper::points3d_in_, "points3d", "The 3d points").required(true);

      outputs.declare(&DepthSwapper::points3d_out_, "points3d", "The modified 3d points");
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      std::vector<cv::Mat> channels;
      cv::split(*points3d_in_, channels);
      if (depth_->depth() == CV_16U)
        depth_->convertTo(channels[2], channels[2].depth(), 0.001);
      else
        depth_->convertTo(channels[2], channels[2].depth());
      cv::merge(channels, *points3d_out_);

      return ecto::OK;
    }
    ecto::spore<cv::Mat> points3d_in_, points3d_out_, depth_;
  };
}

ECTO_CELL(rgbd, rgbd::DepthSwapper, "DepthSwapper", "Changes the z component of some 3d points")
