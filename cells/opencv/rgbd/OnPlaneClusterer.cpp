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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float pointDistanceSq(const cv::Vec3f& vec1, const cv::Vec3f& vec2) {
  cv::Vec3f vec = vec1 - vec2;
  return vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];
}

float pointPlaneDistance(const cv::Vec3f& vec, const cv::Vec4f& plane) {
  return vec[0] * plane[0] + vec[1] * plane[1] + vec[2] * plane[2] + plane[3];
}

cv::Vec3f projectPointOnPlane(const cv::Vec3f& vec, const cv::Vec4f& plane) {
  // sol is vec + alpha*N such that it's on plane (maybe N is not normalized)
  cv::Vec3f N(plane[0], plane[1], plane[2]);
  float alpha = (-plane[3] - vec.dot(N)) / N.dot(N);
  return vec + alpha * N;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class BelongPredicate {
 public:
  virtual ~BelongPredicate() {
  }
  virtual bool operator()(const cv::Vec3f& point,
                          const cv::Vec3f& point_expanded_from,
                          const cv::Vec4f& plane) const = 0;
};

void clusterOnPlane(const cv::Mat_<uchar> &plane_masks,
                    const cv::Mat_<cv::Vec3f> &points3d,
                    const std::vector<cv::Vec4f>&planes,
                    const BelongPredicate& predicate,
                    size_t min_cluster_size,
                    std::vector<std::vector<std::vector<cv::Vec2i> > >&clusters2d,
                    std::vector<std::vector<std::vector<cv::Vec3f> > >&clusters3d) {
  // If an object touches a plane, its pixels also touch some pixels of the plane
  // Let's find those pixels first
  cv::Mat_<uchar> checked = plane_masks != 255, object_seeds;
  cv::dilate(checked, object_seeds, cv::Mat());
  object_seeds = object_seeds - checked;

  // For each potential pixel ...
  clusters2d.clear();
  clusters3d.clear();
  for (int y = 1; y < plane_masks.rows - 1; ++y) {
    uchar* iter = object_seeds.ptr<uchar>(y);
    for (int x = 1; x < plane_masks.cols - 1; ++x, ++iter) {
      // Only look at pixels that are on the edge of planes
      if ((!(*iter)) || (checked(y, x)))
        continue;
      // ok we have a seed, first get the plane from it
      size_t plane_closest = 0;
      for (int yy = y - 1; yy <= y + 1; ++yy)
        for (int xx = x - 1; xx <= x + 1; ++xx) {
          if (plane_masks.at<uchar>(yy, xx) != 255) {
            plane_closest = plane_masks.at<uchar>(yy, xx);
            break;
          }
        }
      // Create a new cluster
      std::vector<cv::Vec2i> cluster2d;
      std::vector<cv::Vec3f> cluster3d;

      // Now, proceed by region growing to find the rest of the object
      std::list<cv::Point> point_list(1, cv::Point(x, y));
      while (!point_list.empty()) {
        // Look at the neighboring points
        const cv::Point& point2d = point_list.front();
        const cv::Vec3f& point3d_1 = points3d(point2d.y, point2d.x);
        // Go over the neighboring points
        for (int yy = std::max(point2d.y - 1, 0);
            yy <= std::min(point2d.y + 1, plane_masks.rows - 1); ++yy)
          for (int xx = std::max(point2d.x - 1, 0);
              xx <= std::min(point2d.x + 1, plane_masks.cols - 1); ++xx) {
            if (checked(yy, xx))
              continue;
            // Compute the distance from that point to the original point
            const cv::Vec3f& point3d_2 = points3d(yy, xx);
            if (predicate(point3d_1, point3d_2, planes[plane_closest])) {
              point_list.push_back(cv::Point(xx, yy));
              cluster2d.push_back(cv::Vec2i(xx, yy));
              cluster3d.push_back(point3d_2);
            }
            checked(yy, xx) = 1;
          }
        point_list.pop_front();
      }

      if (cluster3d.size() < min_cluster_size)
        continue;
      if (plane_closest >= clusters2d.size()) {
        clusters2d.resize(plane_closest + 1);
        clusters3d.resize(plane_closest + 1);
      }
      clusters2d[plane_closest].push_back(cluster2d);
      clusters3d[plane_closest].push_back(cluster3d);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class DistancePredicate : public BelongPredicate {
 public:
  DistancePredicate(float cluster_distance, float z_min, float z_max)
      : cluster_distance_(cluster_distance), z_min_(z_min), z_max_(z_max) {

  }
  bool operator()(const cv::Vec3f& point,
                  const cv::Vec3f& point_expanded_from,
                  const cv::Vec4f& plane) const {
    // Compute the distance from that point to the original point
     if (pointDistanceSq(point, point_expanded_from)
     < cluster_distance_ *cluster_distance_) {
      // Only add the point if it is within the plane distance boundaries
      float dist = pointPlaneDistance(point,plane);
      if ((z_min_ < dist) && (dist < z_max_)) {
        return true;
      }
    }
     return false;
  }
 private:
  float cluster_distance_;
  float z_min_, z_max_;
};

/** Cell that finds the clusters touching planes detected in a given depth image
 */
struct OnPlaneClusterer {
  static void declare_params(ecto::tendrils& params) {
    params.declare(&OnPlaneClusterer::cluster_distance_, "cluster_distance",
        "The maximum distance between a point and the cluster it belongs to.",
        0.02);
    params.declare(&OnPlaneClusterer::min_cluster_size_, "min_cluster_size",
        "Min number of points for a cluster", 300);
    params.declare(&OnPlaneClusterer::z_min_, "z_min",
        "The amount to crop above the plane, in meters.", 0.0075);
    params.declare(&OnPlaneClusterer::z_max_, "z_crop",
        "The amount to keep in the z direction (meters) relative to\n"
            "the coordinate frame defined by the pose.", 0.5);
  }

  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs,
      ecto::tendrils& outputs) {
    inputs.declare(&OnPlaneClusterer::points3d_, "points3d",
        "The 3dpoints as a cv::Mat_<cv::Vec3f>.");
    inputs.declare(&OnPlaneClusterer::masks_, "masks",
        "The masks for each plane.");
    inputs.declare(&OnPlaneClusterer::planes_, "planes",
        "The different found planes (a,b,c,d) of equation ax+by+cz+d=0.");

    outputs.declare(&OnPlaneClusterer::clusters2d_, "clusters2d",
        "For each table, a vector of 2d clusters.");
    outputs.declare(&OnPlaneClusterer::clusters3d_, "clusters3d",
        "For each table, a vector of 3d clusters.");
  }

  /** Get the 2d keypoints and figure out their 3D position from the depth map
   * @param inputs
   * @param outputs
   * @return
   */
  int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs) {
    clusters2d_->clear();
    clusters2d_->resize(planes_->size());
    clusters3d_->clear();
    clusters3d_->resize(planes_->size());

    const cv::Mat_<cv::Vec3f> &points3d = *points3d_;

    clusterOnPlane(*masks_, points3d, *planes_,
                   DistancePredicate(*min_cluster_size_, *z_min_, *z_max_),
                   *min_cluster_size_, *clusters2d_, *clusters3d_);

    return ecto::OK;
  }
private:
  /** Min distance between two clusters */
  ecto::spore<float> cluster_distance_;
  /** Min number of points for a cluster */
  ecto::spore<size_t> min_cluster_size_;
  /** Limits used when clustering points off the plane */
  ecto::spore<float> z_min_;
  ecto::spore<float> z_max_;

  /** The input cloud */
  ecto::spore<cv::Mat> points3d_;
  /** The minimum number of inliers in order to do pose matching */
  ecto::spore<std::vector<cv::Vec4f> > planes_;
  /** The mask of the different planes */
  ecto::spore<cv::Mat> masks_;
  /** The resulting clusters: for each table, return a vector of clusters */
  ecto::spore<std::vector<std::vector<std::vector<cv::Vec2i> > > > clusters2d_;
  ecto::spore<std::vector<std::vector<std::vector<cv::Vec3f> > > > clusters3d_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Cell that finds the cluster of points in a cylinder on top of a plane
 */
struct OnPlaneClustererCylinder {
  static void declare_params(ecto::tendrils& params) {
    params.declare(&OnPlaneClustererCylinder::radius_max_, "radius_crop",
        "The amount to keep in the x direction (meters) relative\n"
            "to the coordinate frame defined by the pose.", 0.2);
    params.declare(&OnPlaneClustererCylinder::z_min_, "z_min",
        "The amount to crop above the plane, in meters.", 0.0075);
    params.declare(&OnPlaneClustererCylinder::z_max_, "z_crop",
        "The amount to keep in the z direction (meters) relative to\n"
            "the coordinate frame defined by the pose.", 0.5);
  }

  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs,
      ecto::tendrils& outputs) {
    inputs.declare(&OnPlaneClustererCylinder::points3d_, "points3d",
        "The 3dpoints as a cv::Mat_<cv::Vec3f>.").required(true);
    inputs.declare(&OnPlaneClustererCylinder::masks_, "masks",
        "The masks for each plane.").required(true);
    inputs.declare(&OnPlaneClustererCylinder::planes_, "planes",
        "The different found planes (a,b,c,d) of equation ax+by+cz+d=0.").required(
        true);
    inputs.declare(&OnPlaneClustererCylinder::T_, "T",
        "The pose translation of the focused plane.").required(true);

    outputs.declare(&OnPlaneClustererCylinder::cluster2d_, "cluster2d",
        "For each table, a vector of 2d clusters.");
    outputs.declare(&OnPlaneClustererCylinder::mask_, "mask",
        "For each table, a vector of 2d clusters.");
    outputs.declare(&OnPlaneClustererCylinder::cluster3d_, "cluster3d",
        "For each table, a vector of 3d clusters.");
  }

  /** Get the 2d keypoints and figure out their 3D position from the depth map
   * @param inputs
   * @param outputs
   * @return
   */
  int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs) {
    cluster2d_->clear();
    cluster3d_->clear();
    cv::Mat_<uchar> mask = cv::Mat_<uchar>::zeros(masks_->size());

    if (planes_->empty() || T_->empty()) {
      *mask_ = mask;
      return ecto::OK;
    }

    // Find the closest plane to the given pose
    cv::Vec3f T = *T_;
    int plane_best_index = 0;
    float distance_best = std::numeric_limits<float>::max();
    for (size_t i = 0; i < planes_->size(); ++i) {
      const cv::Vec4f & plane = (*planes_)[i];
      float dist = pointPlaneDistance(T, plane);
      if (dist < distance_best) {
        distance_best = dist;
        plane_best_index = i;
      }
    }
    const cv::Vec4f &plane_best = (*planes_)[plane_best_index];

    const cv::Mat_<cv::Vec3f> &points3d = *points3d_;

    // If an object touches a plane, its pixels also touch some pixels of the plane
    // Let's find those pixels first
    cv::Mat_<uchar> checked = (*masks_) == plane_best_index, object_seeds;
    cv::dilate(checked, object_seeds, cv::Mat());
    object_seeds = object_seeds - checked;

    // For each potential pixel ...
    for (int y = 0; y < masks_->rows; ++y) {
      uchar* iter = object_seeds.ptr<uchar>(y);
      for (int x = 0; x < masks_->cols; ++x, ++iter) {
        // Only look at pixels that are on the edge of planes
        if ((!(*iter)) || (checked(y, x)))
          continue;
        // Now, proceed by region growing to find the rest of the object
        std::list<cv::Point> point_list(1, cv::Point(x, y));
        while (!point_list.empty()) {
          // Look at the neighboring points
          const cv::Point & point2d = point_list.front();
          for (int yy = std::max(point2d.y - 1, 0); yy <= std::min(point2d.y + 1, masks_->rows - 1); ++yy)
            for (int xx = std::max(point2d.x - 1, 0); xx <= std::min(point2d.x + 1, mask_->cols - 1); ++xx) {
              if (checked(yy, xx))
                continue;
              const cv::Vec3f& point3d = points3d(yy, xx);
              checked(yy, xx) = 255;

              // Only add the point if it is within the plane distance boundaries
              float dist = pointPlaneDistance(point3d, plane_best);
              if ((*z_min_ < dist) && (dist < *z_max_)) {
                // Also make sure it is within the radius boundary
                cv::Vec3f projection = projectPointOnPlane(point3d, plane_best);
                if (cv::norm(T - projection) < *radius_max_) {
                  cluster2d_->push_back(cv::Vec2i(xx, yy));
                  cluster3d_->push_back(point3d);
                  mask(yy, xx) = 255;
                  point_list.push_back(cv::Point(xx, yy));
                }
              }
            }
          point_list.pop_front();
        }
      }
    }
    *mask_ = mask;

    return ecto::OK;
  }
private:
  /** The input cloud */
  ecto::spore<cv::Mat> points3d_;
  /** The minimum number of inliers in order to do pose matching */
  ecto::spore<std::vector<cv::Vec4f> > planes_;
  /** The mask of the different planes */
  ecto::spore<cv::Mat> masks_;
  ecto::spore<float> radius_max_, z_max_, z_min_;
  ecto::spore<cv::Mat> T_;

  /** The resulting cluster */
  ecto::spore<std::vector<cv::Vec2i> > cluster2d_;
  ecto::spore<std::vector<cv::Vec3f> > cluster3d_;
  /** The mask of the 2d points that are in the cylinder */
  ecto::spore<cv::Mat> mask_;
};

ECTO_CELL(rgbd, OnPlaneClusterer, "OnPlaneClusterer",
    "Given masks of planes, find clusters on top of them.");
ECTO_CELL(rgbd, OnPlaneClustererCylinder, "OnPlaneClustererCylinder",
    "Given the mask of a plane, find all the 3d points in a cylinder on top of it.");
