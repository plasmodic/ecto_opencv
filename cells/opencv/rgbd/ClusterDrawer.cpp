/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
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

/** This is an implementation of a visualizer for the output of the table finder: clusters are drawn on an image */

#include <list>
#include <numeric>
#include <set>
#include <string>

#include <opencv2/imgproc/imgproc.hpp>
#if CV_MAJOR_VERSION == 3
#include <opencv2/rgbd.hpp>
#else
#include <opencv2/rgbd/rgbd.hpp>
#endif

#include <ecto/ecto.hpp>

using ecto::tendrils;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  struct ClusterDrawer
  {
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&ClusterDrawer::clusters2d_, "clusters2d", "For each table, a vector of 2d clusters.");
      inputs.declare(&ClusterDrawer::image_, "image", "The image to draw on.").required(true);

      outputs.declare(&ClusterDrawer::image_clusters_, "image", "The depth image with the convex hulls for the planes.");
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      // Draw each cluster
      image_->copyTo(*image_clusters_);
      for(size_t plane_index = 0; plane_index < clusters2d_->size(); ++plane_index) {
        const std::vector<std::vector<cv::Vec2i> > &clusters = (*clusters2d_)[plane_index];
        for(size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index) {
          const std::vector<cv::Vec2i> &cluster = clusters[cluster_index];
          for(size_t i = 0; i < cluster.size(); ++i) {
            // Draw the cluster in white
            (*image_clusters_).at<cv::Vec3b>(cluster[i][1], cluster[i][0]) = cv::Vec3b(0,0,255);
          }
        }
        // Find the contour of the cluster
        // Draw it
        // Draw
      }

      return ecto::OK;
    }
  private:
    /** Input image */
    ecto::spore<cv::Mat> image_;

    /** An output image that contains dense clusters */
    ecto::spore<cv::Mat> image_clusters_;

    /** The resulting clusters: for each table, return a vector of clusters */
    ecto::spore<std::vector<std::vector<std::vector<cv::Vec2i> > > > clusters2d_;
  };

ECTO_CELL(rgbd, ClusterDrawer, "ClusterDrawer", "Draws some clusters in an image.")
