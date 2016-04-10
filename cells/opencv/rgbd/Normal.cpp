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
#include <opencv2/imgproc/imgproc.hpp>
#if CV_MAJOR_VERSION == 3
#include <opencv2/rgbd.hpp>
using cv::rgbd::RgbdNormals;
#else
#include <opencv2/rgbd/rgbd.hpp>
using cv::RgbdNormals;
#endif

using ecto::tendrils;
namespace rgbd
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  struct ComputeNormals
  {
    static void
    declare_params(tendrils & params)
    {
      params.declare(&ComputeNormals::method_, "method", "Conversion type.", RgbdNormals::RGBD_NORMALS_METHOD_FALS);
      params.declare(&ComputeNormals::window_size_, "window_size", "The window size for smoothing.", 5);
    }

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
      if (normals_computer_.empty()) {
        if ((points3d_->depth() == CV_32F) || (points3d_->depth() == CV_64F))
          normals_computer_ = new RgbdNormals(points3d_->rows, points3d_->cols, points3d_->depth(), *K_,
                                                  *window_size_, *method_);
        else
          normals_computer_ = new RgbdNormals(points3d_->rows, points3d_->cols, CV_32F, *K_,
                                                  *window_size_, *method_);
      }
      (*normals_computer_)(*points3d_, *normals_);

      return ecto::OK;
    }
    cv::Ptr<RgbdNormals> normals_computer_;
    ecto::spore<cv::Mat> points3d_, normals_, K_, depth_;
    ecto::spore<RgbdNormals::RGBD_NORMALS_METHOD> method_;
    ecto::spore<int> window_size_;
  };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  template<typename T>
  void fillIntensity(cv::Mat &normals_in, cv::Mat_<uchar> &normal_intensity) {
    cv::Mat_<T> normals(normals_in.rows, normals_in.cols, reinterpret_cast<T*>(normals_in.data));
    for (int y = 0; y < normals.rows; ++y) {
      T *normal = normals[y], *normal_end = normal + normals.cols;
      uchar *intensity = normal_intensity[y];
      for(; normal < normal_end; ++normal, ++intensity) {
        *intensity = 255 * std::abs((*normal)[2]/cv::norm(*normal));
      }
    }
  };

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
      inputs.declare(&DrawNormals::K_, "K", "The intrinsic matrix").required(true);
      inputs.declare(&DrawNormals::points3d_, "points3d", "The 3d points").required(true);
      inputs.declare(&DrawNormals::normals_, "normals", "The normals").required(true);

      outputs.declare(&DrawNormals::image_out_, "image", "The output image");
      outputs.declare(&DrawNormals::normal_intensity_, "normal_intensity",
                      "The output image with the normal intensity");
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      image_in_->copyTo(*image_out_);

      // Draw the normals at given steps
      std::vector<cv::Point3f> points3d;
      std::vector<cv::Point2f> points_beg;
      size_t n_max_points = (image_in_->rows * image_in_->cols) / (*step_) / (*step_);
      points3d.reserve(n_max_points);
      points_beg.reserve(n_max_points);
      for (int y = *step_; y < (image_in_->rows - (*step_)); y += *step_)
        for (int x = *step_; x < (image_in_->cols - (*step_)); x += *step_)
        {
          cv::Vec3f normal;
          if (normals_->depth() == CV_32F)
            normal = normals_->at<cv::Vec3f>(y, x);
          else
            normal = normals_->at<cv::Vec3d>(y, x);
          normal = normal / cv::norm(normal);

          cv::Vec3f point3d;
          if (points3d_->depth() == CV_32F)
            point3d = points3d_->at<cv::Vec3f>(y, x);
          else
            point3d = points3d_->at<cv::Vec3d>(y, x);

          if (cvIsNaN(normal[0]))
            continue;

          points3d.push_back(
              cv::Point3f(point3d[0] + 100 * normal[0], point3d[1] + 100 * normal[1], point3d[2] + 100 * normal[2]));
          points_beg.push_back(cv::Point2f(x, y));
        }

      std::vector<cv::Point2f> points_end;
      cv::Vec3f rvec(0, 0, 0), tvec(0, 0, 0);
      if (!points3d.empty())
        cv::projectPoints(points3d, rvec, tvec, *K_, cv::Mat(), points_end);
      // Normalize the projected normals
      std::vector<float> norms(points_end.size());
      for (size_t i = 0; i < points_end.size(); ++i)
        norms[i] = cv::norm(points_beg[i] - points_end[i]);
      cv::Scalar mean, stddev;

      cv::meanStdDev(norms, mean, stddev);
      float scale = (float(*step_) / 1.2 - 1) / (mean[0] + stddev[0]) * 6;
      for (size_t i = 0; i < points_end.size(); ++i)
      {
        cv::Point2f end_point(points_beg[i].x + (points_end[i].x - points_beg[i].x) * scale,
                              points_beg[i].y + (points_end[i].y - points_beg[i].y) * scale);
        cv::line(*image_out_, points_beg[i], end_point, cv::Scalar(255, 0, 0));
      }

      normal_intensity_->create(image_in_->rows, image_in_->cols, CV_8U);
      cv::Mat_<uchar> normal_intensity(*normal_intensity_);
      if (normals_->depth() == CV_32F)
        fillIntensity<cv::Vec3f>(*normals_, normal_intensity);
      else
        fillIntensity<cv::Vec3d>(*normals_, normal_intensity);

      return ecto::OK;
    }

    ecto::spore<int> step_;
    ecto::spore<cv::Mat> image_in_, image_out_, normals_, points3d_, K_, normal_intensity_;
  };
}

ECTO_CELL(rgbd, rgbd::ComputeNormals, "ComputeNormals", "Compute the normals in a depth image.")
ECTO_CELL(rgbd, rgbd::DrawNormals, "DrawNormals", "Display 3d normals in a depth image.")
