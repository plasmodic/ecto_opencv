/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#pragma once
#ifndef _INTERFACES_HPP
#define _INTERFACES_HPP

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#if (CV_MAJOR_VERSION > 2) || ((CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION >= 4))
#include <opencv2/nonfree/features2d.hpp>
#endif

using ecto::tendrils;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
read_tendrils_as_file_node(const ecto::tendrils & tendrils, boost::function<void
(const cv::FileNode &)> function);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Interface to feature detection, reused by specific feature detectors
 */
struct feature_detector_interface
{
  static void
  declare_outputs(tendrils& outputs)
  {
    outputs.declare<std::vector<cv::KeyPoint> >("keypoints", "The keypoints.");
  }
  static void
  declare_inputs(tendrils& inputs)
  {
    inputs.declare<cv::Mat>("image", "An input image.");
    inputs.declare<cv::Mat>("mask", "An mask, same size as image.");
  }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Interface to descriptor extraction, reused by specific descriptor extractors
 */
struct descriptor_extractor_interface
{
  static void
  declare_outputs(tendrils& outputs)
  {
    outputs.declare<cv::Mat>("descriptors", "The descriptors per keypoints");
  }
  static void
  declare_inputs(tendrils& inputs)
  {
    inputs.declare<cv::Mat>("image", "An input image.");
    inputs.declare<cv::Mat>("mask", "An mask, same size as image.");
    inputs.declare<std::vector<cv::KeyPoint> >("keypoints", "The keypoints.");
    inputs.declare<cv::Mat>("points", "2d points.");

  }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct SIFT_interface
{
  static void
  declare_common_params(tendrils&p);
};

#endif
