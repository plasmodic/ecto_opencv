/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/** This is an implementation of a fast plane detection */

#include <list>
#include <numeric>
#include <set>
#include <string>

#include <opencv2/imgproc/imgproc.hpp>
#if CV_MAJOR_VERSION == 3
#include <opencv2/rgbd.hpp>
using cv::rgbd::RgbdNormals;
using cv::rgbd::RgbdPlane;
#else
#include <opencv2/rgbd/rgbd.hpp>
using cv::RgbdNormals;
using cv::RgbdPlane;
#endif

#include <ecto/ecto.hpp>

using ecto::tendrils;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace rgbd
{
  struct PlaneFinder
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare(&PlaneFinder::block_size_, "block_size",
                     "Size of a block to check if it belongs to a plane (in pixels).", 40);
      params.declare(&PlaneFinder::threshold_, "threshold", "Error (in meters) for how far a point is on a plane.",
                     0.02);
      params.declare(&PlaneFinder::sensor_error_a_, "sensor_error_a",
                     "a coefficient of the quadratic sensor error err=a*z^2+b*z+c. 0.0075 fo Kinect.", 0.0);
      params.declare(&PlaneFinder::sensor_error_b_, "sensor_error_b",
                     "b coefficient of the quadratic sensor error err=a*z^2+b*z+c. 0.0 fo Kinect.", 0.0);
      params.declare(&PlaneFinder::sensor_error_c_, "sensor_error_c",
                     "c coefficient of the quadratic sensor error err=a*z^2+b*z+c. 0.0 fo Kinect.", 0.0);
      params.declare(&PlaneFinder::window_size_, "window_size", "The window size for smoothing.", 5);
      params.declare(&PlaneFinder::normal_method_, "normal_method", "The window size for smoothing.",
                     RgbdNormals::RGBD_NORMALS_METHOD_FALS);
      params.declare(&PlaneFinder::min_size_, "min_size", "The smallest size in pixels of a plane.", 0);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&PlaneFinder::points3d_, "points3d", "The current depth frame.").required(true);
      inputs.declare(&PlaneFinder::K_, "K", "The calibration matrix").required(true);
      inputs.declare(&PlaneFinder::normals_, "normals", "The normals");

      outputs.declare(&PlaneFinder::planes_, "planes",
                      "The different found planes (a,b,c,d) of equation ax+by+cz+d=0.");
      outputs.declare(&PlaneFinder::masks_, "masks", "The masks for each plane.");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      if (normals_->empty())
      {
        if (normals_computer_.empty())
          normals_computer_ = new RgbdNormals(points3d_->rows, points3d_->cols, points3d_->depth(), *K_,
                                                  *window_size_, *normal_method_);
        (*normals_computer_)(*points3d_, *normals_);
      }

      if (plane_computer_.empty())
      {
#if CV_MAJOR_VERSION == 3
        plane_computer_.reset(new RgbdPlane());
#else
        plane_computer_ = cv::Algorithm::create<cv::RgbdPlane>("RGBD.RgbdPlane");
        plane_computer_->set("sensor_error_a", *sensor_error_a_);
        plane_computer_->set("sensor_error_b", *sensor_error_b_);
        plane_computer_->set("sensor_error_c", *sensor_error_c_);
        plane_computer_->set("min_size", *min_size_);
#endif
      }
      (*plane_computer_)(*points3d_, *normals_, *masks_, *planes_);

      return ecto::OK;
    }

  private:
    cv::Ptr<RgbdNormals> normals_computer_;
    cv::Ptr<RgbdPlane> plane_computer_;

    /** If true, display some result */
    ecto::spore<float> threshold_;
    ecto::spore<float> sensor_error_a_;
    ecto::spore<float> sensor_error_b_;
    ecto::spore<float> sensor_error_c_;
    ecto::spore<size_t> block_size_;

    /** Input 3d points */
    ecto::spore<cv::Mat> points3d_;
    ecto::spore<cv::Mat> normals_;

    /** Output planes */
    ecto::spore<std::vector<cv::Vec4f> > planes_;
    /** Output mask of the planes */
    ecto::spore<cv::Mat> masks_;

    ecto::spore<cv::Mat> K_;
    /** The smallest size in pixels of a plane */
    ecto::spore<int> min_size_;

    ecto::spore<int> window_size_;

    ecto::spore<RgbdNormals::RGBD_NORMALS_METHOD> normal_method_;
  };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  struct PlaneDrawer
  {
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&PlaneDrawer::image_, "image", "The current gray frame.").required(true);
      inputs.declare(&PlaneDrawer::masks_, "masks", "The masks for each plane.");

      outputs.declare(&PlaneDrawer::image_clusters_, "image", "The depth image with the convex hulls for the planes.");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      colors_.clear();
      // Get a bunch of colors in HSV
      size_t n_colors = 30;
      cv::Mat_<cv::Vec3b> hsv(n_colors, 1), rgb;
      for (size_t i = 0; i < n_colors; ++i)
        hsv(i) = cv::Vec3b((i * 180) / n_colors, 255, 255);
      cv::cvtColor(hsv, rgb, CV_HSV2RGB);
      for (size_t i = 0; i < n_colors; ++i)
        colors_.push_back(cv::Vec3b(rgb(i)[0], rgb(i)[1], rgb(i)[2]));
      previous_plane_number_ = 0;
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      //// Perform some display
      // Compare each mask to the previous ones
      std::map<int, int> color_index;
      // Compute the current number of planes
      int max_index = -1;
      cv::Mat_<int> overlap = cv::Mat_<int>::zeros(255, previous_plane_number_);
      if (!masks_->empty()) {
        if (!previous_masks_.empty()) {
          // Count the number of current planes as well as the overlap with the previous mask
          for (int y = 0; y < masks_->rows; ++y)
          {
            const unsigned char *mask = masks_->ptr(y), *mask_end = mask + masks_->cols;
            const unsigned char *previous_mask = previous_masks_.ptr(y);
            for (; mask != mask_end; ++mask, ++previous_mask)
            {
              if (*mask != 255) {
                if (*mask > max_index)
                  max_index = *mask;
                if (*previous_mask != 255)
                  ++overlap(*mask, *previous_mask);
              }
            }
          }
        } else {
          // Only count the number of planes
          for (int y = 0; y < masks_->rows; ++y)
          {
            const unsigned char *mask = masks_->ptr(y), *mask_end = mask + masks_->cols;
            for (; mask != mask_end; ++mask)
            {
              if ((*mask != 255) && (*mask > max_index))
                  max_index = *mask;
            }
          }
        }
      }
      int plane_number = max_index + 1;

      if (previous_plane_number_ == 0)
      {
        for (int i = 0; i < plane_number; ++i)
          color_index[i] = i;
      }
      else if (plane_number)
      {
        // Maps a new index to the corresponding old index
        while (true)
        {
          // Find the best overlap
          int max_overlap = 0, max_i = -1, max_j = -1;

          for (int i = 0; i < overlap.rows; ++i)
            for (int j = 0; j < overlap.cols; ++j)
            {
              if (overlap(i, j) > max_overlap)
              {
                max_overlap = overlap(i, j);
                max_i = i;
                max_j = j;
              }
            }

          if (max_overlap == 0)
            break;

          color_index[max_i] = previous_color_index_[max_j];

          // Reset some overlap values
          for (int i = 0; i < overlap.rows; ++i)
            overlap(i, max_j) = 0;

          for (int j = 0; j < overlap.cols; ++j)
            overlap(max_i, j) = 0;
        }

        // Add all the previously used colors
        std::set<int> previously_used_colors;
        for (std::map<int, int>::const_iterator iter = color_index.begin(); iter != color_index.end(); ++iter)
          previously_used_colors.insert(iter->second);

        // Give a color to the blocks that were not assigned
        for (int i = 0; i < overlap.rows; ++i)
        {
          if (color_index.find(i) != color_index.end())
            continue;
          // Look for a color that was not given
          size_t j = 0;
          while (previously_used_colors.find(j) != previously_used_colors.end())
            ++j;

          color_index[i] = j;
          previously_used_colors.insert(j);
        }
      }
      previous_color_index_ = color_index;
      previous_plane_number_ = plane_number;
      masks_->copyTo(previous_masks_);

      // Draw the clusters
      image_->copyTo(*image_clusters_);
      for (int y = 0; y < image_clusters_->rows; ++y)
      {
        cv::Vec3b *image = image_clusters_->ptr<cv::Vec3b>(y);
        const unsigned char *mask = masks_->ptr(y), *mask_end = mask + masks_->cols;
        for (; mask != mask_end; ++image, ++mask)
        {
          if (*mask < 30)
            *image = colors_[color_index[*mask]];
        }
      }

      return ecto::OK;
    }
  private:
    /** Input image */
    ecto::spore<cv::Mat> image_;

    /** Previous size of the output planes */
    size_t previous_plane_number_;
    /** Output mask of the planes */
    ecto::spore<cv::Mat> masks_;

    /** An output image that contains dense clusters */
    ecto::spore<cv::Mat> image_clusters_;

    /** Store the previous resize masks for color consistency */
    cv::Mat previous_masks_;
    /** The previously used indices, for "tracking". Map from color plane index to color index */
    std::map<int, int> previous_color_index_;

    /** The list of colors to use for display */
    std::vector<cv::Vec3b> colors_;
  };
}

ECTO_CELL(rgbd, rgbd::PlaneFinder, "PlaneFinder", "Finds several planes in a depth image.")
ECTO_CELL(rgbd, rgbd::PlaneDrawer, "PlaneDrawer", "Draws planes.")
