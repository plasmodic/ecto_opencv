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
#include <opencv2/rgbd/rgbd.hpp>

using ecto::tendrils;
namespace rgbd
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  struct ComputeNormals
  {
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&ComputeNormals::points3d_, "points3d", "The 3d points from a depth image").required(true);
      inputs.declare(&ComputeNormals::K_, "K", "The calibration matrix").required(true);

      outputs.declare(&ComputeNormals::normals_, "normals", "The normals");
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      if (normals_computer_.empty())
        normals_computer_ = cv::Ptr<cv::RgbdNormals>(
            new cv::RgbdNormals(points3d_->rows, points3d_->cols, points3d_->depth(), *K_));
      *normals_ = (*normals_computer_)(*points3d_);

      return ecto::OK;
    }
    cv::Ptr<cv::RgbdNormals> normals_computer_;
    ecto::spore<cv::Mat> points3d_, normals_, K_;
  };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  struct DrawNormals
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare(&DrawNormals::step_, "step", "The step at which to display normals in pixels.", 40);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&DrawNormals::image_in_, "image", "The input image").required(true);
      inputs.declare(&DrawNormals::normals_, "normals", "The normals").required(true);

      outputs.declare(&DrawNormals::image_out_, "image", "The output image");
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      image_in_->copyTo(*image_out_);

      // Draw the normals at given steps
      int len = (*step_) / 2 - 2;
      for (int y = *step_; y < (image_in_->rows - (*step_)); y += *step_)
        for (int x = *step_; x < (image_in_->cols - (*step_)); x += *step_)
        {
          float angle;
          if (normals_->depth() == CV_32F)
            angle = std::atan2(normals_->at<cv::Vec3f>(y, x)[1], normals_->at<cv::Vec3f>(y, x)[0]);
          else
            angle = std::atan2(normals_->at<cv::Vec3d>(y, x)[1], normals_->at<cv::Vec3d>(y, x)[0]);

          std::cout << normals_->at<cv::Vec3f>(y, x)[0] << " " << normals_->at<cv::Vec3f>(y, x)[1] << " "
                    << normals_->at<cv::Vec3f>(y, x)[1] << std::endl;
          if (cvIsNaN(angle))
            continue;

          cv::line(*image_out_, cv::Point2i(x, y), cv::Point2i(x + len * std::cos(angle), y - len * std::sin(angle)),
                   cv::Scalar(0, 0, 255));
        }

      return ecto::OK;
    }

    ecto::spore<int> step_;
    ecto::spore<cv::Mat> image_in_, image_out_, normals_;
  };
}

ECTO_CELL(rgbd, rgbd::ComputeNormals, "ComputeNormals", "Compute the normals in a depth image.")
ECTO_CELL(rgbd, rgbd::DrawNormals, "DrawNormals", "Display 3d normals in a depth image.")
