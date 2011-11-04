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
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using ecto::tendrils;
namespace calib
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void
  depth_mask(const cv::Mat& input, cv::Mat& output)
  {
    if (input.depth() != CV_32F)
      throw std::runtime_error("Expected input to be of floating point value, CV_32F");
    output.create(input.size(), CV_8UC1);
    output = cv::Scalar::all(255);
    cv::Mat_<float>::const_iterator begin(input.begin<float>()), end(input.end<float>());
    cv::Mat_<uint8_t>::iterator out_i(output.begin<uint8_t>());
    while (begin != end)
    {
      float v = *begin;
      *out_i = (!std::isnan(v)) * 255;
      ++out_i;
      ++begin;
    }
  }
  struct DepthMask
  {

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&DepthMask::depth, "depth", "The depth image").required(true);
      outputs.declare(&DepthMask::mask, "mask", "Valid points");
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      *mask = cv::Mat();
      depth_mask(*depth, *mask);
      return ecto::OK;
    }
    ecto::spore<cv::Mat> depth, mask;
  };

  struct DepthValidDraw
  {
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&DepthValidDraw::image, "image", "The image").required(true);
      inputs.declare(&DepthValidDraw::mask, "mask", "The depth mask").required(true);
      outputs.declare(&DepthValidDraw::result, "image", "Valid areas of the image.");
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      *result = cv::Mat(image->size(), image->type());
      *result = cv::Scalar(0, 0, 255);
      cv::bitwise_and(*image, *image, *result, *mask);
      return ecto::OK;
    }
    ecto::spore<cv::Mat> image, result, mask;
  };
}
using namespace calib;
ECTO_CELL(calib, DepthMask, "DepthMask", "Converts depth map to a mask, assuming that invalid points are == 0.")
ECTO_CELL(calib, DepthValidDraw, "DepthValidDraw", "DepthValidDraw q.")
