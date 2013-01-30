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

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/rgbd/rgbd.hpp>

using ecto::tendrils;

float pointDistanceSq(const cv::Vec3f& vec1, const cv::Vec3f& vec2)
{
  cv::Vec3f vec = vec1 - vec2;
  return vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];
}

float pointPlaneDistance(const cv::Vec3f& vec, const cv::Vec4f& plane)
{
  return std::abs(vec[0] * plane[0] + vec[1] * plane[1] + vec[2] * plane[2] + plane[3]);
}

  /** Cell that finds the clusters touching planes detected in a given depth image
   */
  struct OnPlaneClusterer
  {
    static void
    declare_params(ecto::tendrils& params)
    {
      params.declare(&OnPlaneClusterer::cluster_distance_, "cluster_distance", "The maximum distance between a point and the cluster it belongs to.",
                     0.02);
      params.declare(&OnPlaneClusterer::min_cluster_size_, "min_cluster_size", "Min number of points for a cluster", 300);
      params.declare(&OnPlaneClusterer::table_z_filter_min_, "table_z_filter_min",
                     "Min distance (in meters) from the table to get clusters from.", 0.01);
      params.declare(&OnPlaneClusterer::table_z_filter_max_, "table_z_filter_max",
                     "Max distance (in meters) from the table to get clusters from.", 0.5);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&OnPlaneClusterer::points3d_, "points3d", "The 3dpoints as a cv::Mat_<cv::Vec3f>.");
      inputs.declare(&OnPlaneClusterer::mask_, "table_mask", "The mask of the different planes.");
      inputs.declare(&OnPlaneClusterer::table_coefficients_, "table_coefficients", "The coefficients of planar surfaces.");

      outputs.declare(&OnPlaneClusterer::clusters2d_, "clusters2d", "For each table, a vector of 2d clusters.");
      outputs.declare(&OnPlaneClusterer::clusters3d_, "clusters3d", "For each table, a vector of 3d clusters.");
    }

    /** Get the 2d keypoints and figure out their 3D position from the depth map
     * @param inputs
     * @param outputs
     * @return
     */
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
    clusters2d_->clear();
    clusters2d_->resize(table_coefficients_->size());
    clusters3d_->clear();
    clusters3d_->resize(table_coefficients_->size());

    const cv::Mat_<cv::Vec3f> &points3d = *points3d_;

    // If an object touches a plane, its pixels also touch some pixels of the plane
    // Let's find those pixels first
    cv::Mat_<uchar> mask_binary = (*mask_) != 255, object_seeds;
    cv::dilate(mask_binary, object_seeds, cv::Mat());
    object_seeds = object_seeds - mask_binary;

    // For each potential pixel ...
    cv::Mat_<uchar> checked = mask_binary.clone();
    for (int y = 1; y < mask_->rows - 1; ++y) {
      uchar* iter = object_seeds.ptr<uchar>(y);
      for (int x = 1; x < mask_->cols - 1; ++x, ++iter) {
        // Only look at pixels that are on the edge of planes
        if ((!(*iter)) || (checked(y, x)))
          continue;
        // ok we have a seed, first get the plane from it
        int plane_closest = -1;
        float plane_distance_min = std::numeric_limits<float>::max();
        for (int yy = y - 1; yy <= y + 1; ++yy)
          for (int xx = x - 1; xx <= x + 1; ++xx) {
            if ((*mask_).at<uchar>(yy, xx) != 255) {
              // compute the distance from the point to the plane
              float plane_distance = pointDistanceSq(points3d(y, x), points3d(yy, xx));
              if (plane_distance < plane_distance_min) {
                plane_closest = (*mask_).at<uchar>(yy, xx);
                plane_distance_min = plane_distance;
              }
            }
          }
        if (plane_closest < 0)
          continue;
        // Create a new cluster
        std::vector<cv::Vec2f> cluster2d;
        std::vector<cv::Vec3f> cluster3d;

        // Now, proceed by region growing to find the rest of the object
        std::list<cv::Point> point_list(1, cv::Point(x, y));
        while (!point_list.empty()) {
          // Look at the neighboring points
          const cv::Point& point2d = point_list.front();
          const cv::Vec3f& point3d_1 = points3d(point2d.y, point2d.x);
          for (int yy = point2d.y - 1; yy <= point2d.y + 1; ++yy)
            for (int xx = point2d.x - 1; xx <= point2d.x + 1; ++xx) {
              if (checked(yy, xx))
                continue;
              // Compute the distance from that point to the original point
              const cv::Vec3f& point3d_2 = points3d(yy, xx);
              if (pointDistanceSq(point3d_1, point3d_2) < (*cluster_distance_) * (*cluster_distance_)) {
                checked(yy, xx) = 1;
                point_list.push_back(cv::Point(xx, yy));
                // Only add the point if it is within the plane distance boundaries
                float dist = pointPlaneDistance(point3d_2, (*table_coefficients_)[plane_closest]);
                if ((*table_z_filter_min_ < dist) && (dist < *table_z_filter_max_))
                  cluster3d.push_back(point3d_2);
              }
            }
          point_list.pop_front();
        }

        if (cluster3d.size() < 100)
          continue;
        (*clusters2d_)[plane_closest].push_back(cluster2d);
        (*clusters3d_)[plane_closest].push_back(cluster3d);
      }
    }

    return ecto::OK;
  }
  private:
    /** Min distance between two clusters */
    ecto::spore<float> cluster_distance_;
    /** Min number of points for a cluster */
    ecto::spore<int> min_cluster_size_;
    /** Limits used when clustering points off the plane */
    ecto::spore<float> table_z_filter_min_;
    ecto::spore<float> table_z_filter_max_;

    /** The input cloud */
    ecto::spore<cv::Mat> points3d_;
    /** The minimum number of inliers in order to do pose matching */
    ecto::spore<std::vector<cv::Vec4f> > table_coefficients_;
    /** The mask of the different planes */
    ecto::spore<cv::Mat> mask_;
    /** The resulting clusters: for each table, return a vector of clusters */
    ecto::spore<std::vector<std::vector<std::vector<cv::Vec2f> > > > clusters2d_;
    ecto::spore<std::vector<std::vector<std::vector<cv::Vec3f> > > > clusters3d_;
  };

ECTO_CELL(rgbd, OnPlaneClusterer, "OnPlaneClusterer",
          "Given a point cloud and the hull of the table, find clusters.");
