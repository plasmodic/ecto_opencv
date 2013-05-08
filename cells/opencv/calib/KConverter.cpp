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

using ecto::tendrils;

struct KConverter {
  typedef KConverter C;

  static void declare_io(const tendrils& params, tendrils& inputs,
      tendrils& outputs) {
    inputs.declare(&C::image_, "image", "The initial image.").required(true);
    inputs.declare(&C::depth_, "depth", "The initial image.").required(true);
    inputs.declare(&C::K_image_, "K_image",
        "The calibration matrix to transform.").required(true);

    outputs.declare(&C::K_depth_, "K_depth", "The calibration matrix.");
  }

  int process(const tendrils& inputs, const tendrils& outputs) {
    K_image_->copyTo(*K_depth_);
    double ratio_x = double(depth_->cols / 2) / double(image_->cols / 2);
    double ratio_y = double(depth_->rows / 2) / double(image_->rows / 2);
    if (K_image_->depth() == CV_32F) {
      K_depth_->at<float>(0, 0) *= ratio_x;
      K_depth_->at<float>(0, 2) *= ratio_x;
      K_depth_->at<float>(1, 1) *= ratio_y;
      K_depth_->at<float>(1, 2) *= ratio_y;
    } else {
      K_depth_->at<double>(0, 0) *= ratio_x;
      K_depth_->at<double>(0, 2) *= ratio_x;
      K_depth_->at<double>(1, 1) *= ratio_y;
      K_depth_->at<double>(1, 2) *= ratio_y;
    }

    return ecto::OK;
  }
  ecto::spore<cv::Mat> image_, depth_, K_image_, K_depth_;
};

ECTO_CELL(calib, KConverter, "KConverter",
    "Converts a calibration matrix to a different image size.")
