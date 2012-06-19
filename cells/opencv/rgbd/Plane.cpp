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

#include <opencv2/rgbd/rgbd.hpp>

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
      params.declare(&PlaneFinder::n_inliers_, "n_inliers", "Number of inliers to consider to define a plane.", 50);
      params.declare(&PlaneFinder::n_samples_, "n_samples", "Number of samples to draw to check if we have a plane.",
                     300);
      params.declare(&PlaneFinder::n_trials_, "n_trials", "Number of trials to make to find a plane.", 100);
      params.declare(&PlaneFinder::error_, "error", "Error (in meters) for how far a point is on a plane.", 0.02);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&PlaneFinder::points3d_, "point3d", "The current depth frame.").required(true);

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
      cv::RgbdPlane rgb_plane;
      rgb_plane(*points3d_, *masks_, *planes_);

      return ecto::OK;
    }

  private:
    /** If true, display some result */
    ecto::spore<float> error_;
    ecto::spore<size_t> block_size_;
    ecto::spore<size_t> n_inliers_;
    ecto::spore<size_t> n_samples_;
    ecto::spore<size_t> n_trials_;

    /** Input 3d points */
    ecto::spore<cv::Mat> points3d_;

    /** Output planes */
    ecto::spore<std::vector<cv::Vec4f> > planes_;
    /** Output mask of the planes */
    ecto::spore<cv::Mat> masks_;
  };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  struct PlaneDrawer
  {
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&PlaneDrawer::image_, "image", "The current gray frame.").required(true);
      inputs.declare(&PlaneDrawer::planes_, "planes", "The different found planes (a,b,c,d) of equation ax+by+cz+d=0.");
      inputs.declare(&PlaneDrawer::masks_, "masks", "The masks for each plane.");

      outputs.declare(&PlaneDrawer::image_clusters_, "image", "The depth image with the convex hulls for the planes.");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      colors_.clear();
      colors_.push_back(cv::Scalar(255, 255, 0));
      colors_.push_back(cv::Scalar(0, 255, 255));
      colors_.push_back(cv::Scalar(255, 0, 255));
      colors_.push_back(cv::Scalar(255, 0, 0));
      colors_.push_back(cv::Scalar(0, 255, 0));
      colors_.push_back(cv::Scalar(0, 0, 255));
      colors_.push_back(cv::Scalar(0, 0, 0));
      colors_.push_back(cv::Scalar(85, 85, 85));
      colors_.push_back(cv::Scalar(170, 170, 170));
      colors_.push_back(cv::Scalar(255, 255, 255));
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      //// Perform some display
      // Compare each mask to the previous ones
      std::map<int, int> color_index;
      if (previous_planes_.empty())
      {
        for (size_t i = 0; i < planes_->size(); ++i)
          color_index[i] = i;
      }
      else if (!planes_->empty())
      {
        cv::Mat_<int> overlap = cv::Mat_<int>::zeros(planes_->size(), previous_planes_.size());

        for (int y = 0; y < masks_->rows; ++y)
        {
          const unsigned char *mask = masks_->ptr(y), *mask_end = masks_->ptr(y) + masks_->cols;
          const unsigned char *previous_mask = previous_masks_.ptr(y);
          for (; mask != mask_end; ++mask, ++previous_mask)
            if ((*mask) && (*previous_mask))
              ++overlap(*mask, *previous_mask);
        }

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
        for (size_t i = 0; i < previous_color_index_.size(); ++i)
          previously_used_colors.insert(previous_color_index_[i]);

        // Give a color to the blocks that were not assigned
        for (int i = 0; i < overlap.rows; ++i)
        {
          // Look for a color that was not given
          size_t j = 0;
          while (previously_used_colors.find(j) != previously_used_colors.end())
            ++j;

          color_index[i] = j;
          previously_used_colors.insert(j);
        }

        previous_color_index_ = color_index;
        previous_planes_ = *planes_;
      }

      // Draw the clusters
      image_->copyTo(*image_clusters_);
      for (size_t i = 0; i < planes_->size(); ++i)
      {
        cv::Mat mask;
        cv::compare(*masks_, cv::Scalar(i), mask, cv::CMP_EQ);
        image_clusters_->setTo(colors_[color_index[i]], mask);
      }

      return ecto::OK;
    }
  private:
    /** Input image */
    ecto::spore<cv::Mat> image_;

    /** Output planes */
    ecto::spore<std::vector<cv::Vec4f> > planes_;
    /** Previous output planes */
    std::vector<cv::Vec4f> previous_planes_;
    /** Output mask of the planes */
    ecto::spore<cv::Mat> masks_;

    /** An output image that contains dense clusters */
    ecto::spore<cv::Mat> image_clusters_;

    /** Store the previous resize masks for color consistency */
    cv::Mat previous_masks_;
    /** The previously used indices, for "tracking". Map from color plane index to color index */
    std::map<int, int> previous_color_index_;

    /** The list of colors to use for display */
    std::vector<cv::Scalar> colors_;
  };
}

ECTO_CELL(rgbd, rgbd::PlaneFinder, "PlaneFinder", "Finds several planes in a depth image.")
ECTO_CELL(rgbd, rgbd::PlaneDrawer, "PlaneDrawer", "Draws planes.")
