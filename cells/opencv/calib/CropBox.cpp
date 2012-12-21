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

namespace calib
{
    struct CropBox
    {
      static void
      declare_params(ecto::tendrils& params)
      {
  		params.declare(&CropBox::enabled_, "crop_enabled", "If the cropper cell is enabled", true);

  		params.declare(&CropBox::x_min_, "x_min", "The minimum x value (in the camera reference frame)", -std::numeric_limits<float>::max());
  		params.declare(&CropBox::x_max_, "x_max", "The maximum x value (in the camera reference frame)", std::numeric_limits<float>::max());
  		params.declare(&CropBox::y_min_, "y_min", "The minimum y value (in the camera reference frame)", -std::numeric_limits<float>::max());
  		params.declare(&CropBox::y_max_, "y_max", "The maximum y value (in the camera reference frame)", std::numeric_limits<float>::max());
  		params.declare(&CropBox::z_min_, "z_min", "The minimum z value (in the camera reference frame)", -std::numeric_limits<float>::max());
  		params.declare(&CropBox::z_max_, "z_max", "The maximum z value (in the camera reference frame)", std::numeric_limits<float>::max());
      }

      void
      configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {
      }

      static void
      declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
      {
    	inputs.declare(&CropBox::rgb_, "rgb", "The rgb image");
    	inputs.declare(&CropBox::depth_, "depth", "The depth image");
        inputs.declare(&CropBox::points3d_, "points3d", "The 3d points: width by height by 3 channels");

        outputs.declare(&CropBox::mask_, "mask", "The mask of what is within the depth range in the image");
        outputs.declare(&CropBox::out_rgb_, "rgb", "The rgb image");
        outputs.declare(&CropBox::out_depth_, "depth", "The depth image");
        outputs.declare(&CropBox::out_points3d_, "points3d", "The 3d points: width by height by 3 channels");
      }

      int
      process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {
    	if(*enabled_)
    	{
			// Get the depth
			std::vector<cv::Mat> channels(3);
			cv::split(*points3d_, channels);
			cv::Mat mask = (
								(*x_min_ < channels[0]) & (channels[0] < *x_max_) &
								(*y_min_ < channels[1]) & (channels[1] < *y_max_) &
								(*z_min_ < channels[2]) & (channels[2] < *z_max_)
							);

			mask.copyTo(*mask_);

			rgb_->copyTo(*out_rgb_, mask);
			depth_->copyTo(*out_depth_, mask);
			points3d_->copyTo(*out_points3d_, mask);

			// Set zeros to NaN
			mask = ~mask;
			out_depth_->setTo(cv::Scalar(std::numeric_limits<float>::quiet_NaN()), mask);
			out_points3d_->setTo(cv::Scalar(std::numeric_limits<float>::quiet_NaN()), mask);
    	}
    	else
    	{
    		*mask_ = cv::Mat(rgb_->size(), CV_8UC1, cv::Scalar(255));
    		*out_rgb_ = *rgb_;//->clone();
    		*out_depth_ = *depth_;//->clone();
    		*out_points3d_ = *points3d_;//->clone();
    	}

        return ecto::OK;
      }
	private:
	  // Params
	  ecto::spore<bool> enabled_;
	  ecto::spore<float> x_min_;
	  ecto::spore<float> x_max_;
	  ecto::spore<float> y_min_;
	  ecto::spore<float> y_max_;
	  ecto::spore<float> z_min_;
	  ecto::spore<float> z_max_;

	  // I/O
	  ecto::spore<const cv::Mat> rgb_, depth_, points3d_;
	  ecto::spore<cv::Mat> mask_, out_rgb_, out_depth_, out_points3d_;
    };
}

ECTO_CELL(calib, calib::CropBox, "CropBox",
          "Given an rgb, a depth image and a points3d matrix, returns only what is inside a bounding box.")
