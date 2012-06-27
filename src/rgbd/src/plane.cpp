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

#include <iostream>
#include <list>
#include <numeric>
#include <set>
#include <string>
//#include <valgrind/callgrind.h>

#include <boost/foreach.hpp>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/rgbd/rgbd.hpp>

/** Structure defining a plane */
struct Plane
{
  /**
   * @param p homogeneous points
   * @param index
   */
  Plane(const std::vector<cv::Vec4f> & p4, int index)
      :
        index_(index)
  {
    std::vector<cv::Vec3f> p(3);

    for (unsigned char i = 0; i < 3; ++i)
      p[i] = cv::Vec3f(p4[i][0], p4[i][1], p4[i][2]);

    cv::Vec3f r = (p[1] - p[0]).cross(p[2] - p[0]);
    r = r / cv::norm(r);
    abcd_[0] = r[0];
    abcd_[1] = r[1];
    abcd_[2] = r[2];
    abcd_[3] = -p[0].dot(r);
  }

  inline
  float
  distance(const cv::Vec3f& p_j) const
  {
    return std::abs(abcd_[0] * p_j[0] + abcd_[1] * p_j[1] + abcd_[2] * p_j[2] + abcd_[3]);
  }

  /** coefficients for ax+by+cz+d = 0 */
  cv::Vec4f abcd_;
  /** The index of the plane */
  int index_;
};

std::ostream&
operator<<(std::ostream& out, const Plane& plane)
{
  out << "coeff: " << plane.abcd_[0] << "," << plane.abcd_[1] << "," << plane.abcd_[2] << "," << plane.abcd_[3] << ",";
  return out;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** A structure that contains the masks related to the different planes. Not used yet */
class PlaneMasks
{
  PlaneMasks(int rows, int cols, int block_size)
      :
        mask_(cv::Mat_<uchar>::zeros(rows, cols))
  {
    int mini_rows = rows / block_size;

    if (rows % block_size != 0)
      ++mini_rows;

    int mini_cols = cols / block_size;

    if (cols % block_size != 0)
      ++mini_cols;

    mini_mask_ = cv::Mat_<uchar>::zeros(mini_rows, mini_cols);
  }

  // Mask of size width x height: 1 were a point belongs to a plane, 0 otherwise
  cv::Mat_<uchar> mask_;
  // Same as mask but of size (width/block_size) x (height/block_size)
  cv::Mat_<uchar> mini_mask_;
  // A list of masks width x height: one per plane. The content: 0 if the point is not on the plane,
  // plane_index if it is
  std::vector<cv::Mat_<uchar> > masks_;
  // Same as masks but of size (width/block_size) x (height/block_size)
  std::vector<cv::Mat_<uchar> > mini_masks_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** A structure that contains the mask of a plane, as well as extra info */
class PlaneMask
{
public:
  PlaneMask(int rows, int cols, int block_size)
      :
        block_size_(block_size),
        mask_(cv::Mat_<uchar>::zeros(rows, cols))
  {
    int mini_rows = rows / block_size;

    if (rows % block_size != 0)
      ++mini_rows;

    int mini_cols = cols / block_size;

    if (cols % block_size != 0)
      ++mini_cols;

    mask_mini_ = cv::Mat_<uchar>::zeros(mini_rows, mini_cols);
  }

  int
  block_size() const
  {
    return block_size_;
  }

  cv::Mat&
  mask()
  {
    return mask_;
  }

  const cv::Mat&
  mask() const
  {
    return mask_;
  }

  const cv::Mat&
  mask_mini() const
  {
    return mask_mini_;
  }

  const cv::Range&
  range_x() const
  {
    return range_x_;
  }

  const cv::Range&
  range_y() const
  {
    return range_y_;
  }

  const uchar&
  operator()(int y, int x) const
  {
    return mask_(y, x);
  }

  uchar&
  at_mini(int y, int x)
  {
    return mask_mini_(y, x);
  }

  bool
  set(const cv::Range& range_y, const cv::Range& range_x, int refinement_iteration, const cv::Mat& good_points,
      int& n_inliers)
  {
    cv::Mat_<uchar> sub_mask = mask_(range_y, range_x);
    sub_mask = sub_mask | good_points;

    n_inliers = cv::countNonZero(good_points);
    size_t n_points = range_x.size() * range_y.size();

    bool is_good = n_inliers > int(n_points / 2);

    if (is_good)
    {
      mask_mini_(range_y.start / block_size_, range_x.start / block_size_) = refinement_iteration;

      // Update the ranges of the mask, for sampling when refining the plane
      if (range_x_.size() == 0)
      {
        range_x_ = range_x;
        range_y_ = range_y;
      }
      else
      {
        if (range_x.start < range_x_.start)
          range_x_.start = range_x.start;
        else if (range_x.end > range_x_.end)
          range_x_.end = range_x.end;

        if (range_y.start < range_y_.start)
          range_y_.start = range_y.start;
        else if (range_y.end > range_y_.end)
          range_y_.end = range_y.end;
      }
    }

    return is_good;
  }

private:
  /** The size of the block */
  int block_size_;
  /** Mask of size width x height: 1 were a point belongs to a plane, 0 otherwise */
  cv::Mat_<uchar> mask_;
  /** Same as mask but of size (width/block_size) x (height/block_size).
   * 0, when the block does not belong to the plane or has not been studied yet
   * 1 when most of the block belongs to the plane
   * 255 when the block is on the contour and is either not studied yet, or not on the plane
   * Otherwise, the refinement iteration number if the block is on the plane
   */
  cv::Mat_<uchar> mask_mini_;
  /** The bounding box of the mask in mask_ */
  cv::Range range_x_;
  cv::Range range_y_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int
PointDistanceSq(const cv::Point2i& point_1, const cv::Point2i& point_2)
{
  return (point_1.x - point_2.x) * (point_1.x - point_2.x) + (point_1.y - point_2.y) * (point_1.y - point_2.y);
}

class Sampler
{
public:
  Sampler(const cv::Mat& points3d, const cv::Mat_<uchar> & overall_mask)
      :
        points3d_(points3d),
        overall_mask_(overall_mask)
  {
  }

  bool
  Find(std::vector<cv::Point2i> & points) const
  {
    static cv::RNG rng;
    points.resize(3);
    std::vector<cv::Point2i> final_points;

    size_t n_iter = 100;
    int min_edge = 50, max_edge = 250, edge_increment = 20;
    int triangle_height_min = float(min_edge) * 1.73 / 2;

    bool has_good_sample = false;

    for (size_t i_iter = 0; i_iter < n_iter; ++i_iter)
    {
      // Get the top tip of the triangle
      points[0].x = rng.uniform(min_edge + 1, points3d_.cols - min_edge - 1);
      points[0].y = rng.uniform(1, points3d_.rows - triangle_height_min - 1);

      // Try several scales
      for (float edge = min_edge; edge < max_edge; edge += edge_increment)
      {
        points[1].x = points[0].x - edge / 2;
        points[1].y = points[0].y + float(edge) * 1.73 / 2;
        points[2].x = points[0].x + edge / 2;
        points[2].y = points[1].y;

        if (!Validate(points))
        {
          bool is_good = false;
          std::vector<cv::Point2i> points_shifted(3);

          // Look for shifted version of the triangle that could be ok
          for (int y_shifted = -edge_increment; (y_shifted <= edge_increment) && (!is_good); y_shifted +=
              edge_increment)
            for (int x_shifted = -edge_increment; x_shifted <= edge_increment; x_shifted += edge_increment)
            {
              points_shifted = points;

              for (unsigned char i = 0; i < 3; ++i)
              {
                points_shifted[i].x = points[i].x + x_shifted;
                points_shifted[i].y = points[i].y + y_shifted;
              }

              if (Validate(points_shifted))
              {
                points = points_shifted;
                is_good = true;
                break;
              }
            }

          if (!is_good)
            break;
        }

        final_points = points;
        has_good_sample = true;
      }

      if (has_good_sample)
        break;
    }

    points = final_points;

    return has_good_sample;
  }
private:
  bool
  Validate(std::vector<cv::Point2i> &points) const
  {
    for (unsigned char i = 0; i < 3; ++i)
    {
      // Make sure we are within the image with a boundary size big enough to look at neighbors
      if ((points[i].x < 1) || (points[i].x >= points3d_.cols - 1) || (points[i].y < 1)
          || (points[i].y >= points3d_.rows - 1))
        return false;

      // Check that the point is valid
      if ((!overall_mask_(points[i].y, points[i].x))
          && (!cvIsNaN(points3d_.at<cv::Vec3f>(points[i].y, points[i].x)[0])))
        continue;

      // If invalid, try to find neighbors that fit the criteria
      bool is_good = false;

      for (int y = points[i].y - 1; (y != points[i].y + 1) && (!is_good); ++y)
        for (int x = points[i].x - 1; x != points[i].x + 1; ++x)
          if ((!cvIsNaN(points3d_.at<cv::Vec3f>(y, x)[i])) && (!overall_mask_(y, x)))
          {
            points[i] = cv::Point2i(x, y);
            is_good = true;
            break;
          }

      if (!is_good)
        return false;
    }

    return true;
  }
  const cv::Mat& points3d_;
  const cv::Mat_<uchar> & overall_mask_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class InlierFinder
{
public:
  InlierFinder(float err, const Plane& plane, const cv::Mat& points3d, cv::Mat_<uchar> &overall_mask, PlaneMask& mask)
      :
        err_(err),
        plane_(plane),
        points3d_(points3d),
        overall_mask_(overall_mask),
        mask_(mask),
        refinement_iteration_(0)
  {
  }

  void
  Find(const cv::Point2i& d_0, int& n_inliers_total)
  {
    std::list<cv::Point2i> blocks;
    ++refinement_iteration_;

    // Add the first block to start from and its neighbors
    cv::Point2i block(d_0.x / mask_.block_size(), d_0.y / mask_.block_size());

    for (int y = std::max(0, block.y - 1); y <= std::min(block.y + 1, mask_.mask_mini().rows - 1); ++y)
      for (int x = std::max(0, block.x - 1); x <= std::min(block.x + 1, mask_.mask_mini().cols - 1); ++x)
      {
        if (mask_.at_mini(y, x) == 0)
          mask_.at_mini(y, x) = 255;
        else
          mask_.at_mini(y, x) = refinement_iteration_;

        blocks.push_back(cv::Point2i(x, y));
      }

    Find(blocks, n_inliers_total);
  }

  void
  Find(int& n_inliers_total)
  {
    std::list<cv::Point2i> blocks;
    ++refinement_iteration_;
    Find(blocks, n_inliers_total);
  }

private:

  /** Given a small rectangle to study, check how many plane inliers there are, and set them in the mask
   * @param point3d it is already a submatrix of the original points3d it is h x w x 3
   * @param r
   * @param p0
   * @return
   */
  bool
  IsBlockOnPlane(const cv::Point2i& block, int& n_inliers)
  {
    cv::Mat point3d_reshape;
    cv::Range range_x, range_y;
    int x = block.x * mask_.block_size(), y = block.y * mask_.block_size();

    if (block.x == mask_.mask_mini().cols - 1)
      range_x = cv::Range(x, mask_.mask().cols);
    else
      range_x = cv::Range(x, x + mask_.block_size());

    if (block.y == mask_.mask_mini().rows - 1)
      range_y = cv::Range(y, mask_.mask().rows);
    else
      range_y = cv::Range(y, y + mask_.block_size());

    cv::Mat good_points;
    bool is_valid;

    if ((mask_.at_mini(y, x) != 255) && 0)
    {
      // If the block already belongs to the plane, only process the points that were not on it
      for (int yy = range_y.start; yy != range_y.end; ++yy)
      {
        uchar* data = mask_.mask().ptr(yy, range_x.start), *data_end = mask_.mask().ptr(yy, range_x.end);
        const cv::Vec3f* point = points3d_.ptr<cv::Vec3f>(yy, range_x.start);

        for (; data != data_end; ++data, ++point)
        {
          if (*data)
            continue;

          if (plane_.distance(*point) < err_)
            *data = 1;
        }
      }

      good_points = mask_.mask()(range_y, range_x);

      // The block was valid the previous iteration so it is still valid
      mask_.set(range_y, range_x, refinement_iteration_, good_points, n_inliers);
      is_valid = true;
    }
    else
    {
      points3d_(range_y, range_x).copyTo(point3d_reshape);
      size_t n_points = point3d_reshape.cols * point3d_reshape.rows;
      // Make the matrix cols*ros x 3
      point3d_reshape = point3d_reshape.reshape(1, n_points);

      cv::Mat errs = point3d_reshape * (cv::Mat_<float>(3, 1) << plane_.abcd_[0], plane_.abcd_[1], plane_.abcd_[2]);

      // Find the points on the plane
      good_points = (-err_ - plane_.abcd_[3] < errs) & (errs < err_ - plane_.abcd_[3]);
      good_points = good_points.reshape(1, range_y.size());

      // Fill the mask with the valid points
      is_valid = mask_.set(range_y, range_x, refinement_iteration_, good_points, n_inliers);
    }

    cv::Mat sub_mask = overall_mask_(range_y, range_x);
    sub_mask.setTo(cv::Scalar(plane_.index_), good_points);

    // Half is totally arbitrary: seems like a good number plus we have NaN's
    return is_valid;
  }

  void
  Find(std::list<cv::Point2i> &blocks, int& n_inliers_total)
  {
    // Also process the contour blocks
    blocks.insert(blocks.end(), contour_blocks_.begin(), contour_blocks_.end());
    contour_blocks_.clear();

    while (!blocks.empty())
    {
      cv::Point2i block = blocks.front();
      blocks.pop_front();

      // Don't look at the neighboring blocks if this is not a fitting block and leave it as 255
      int n_inliers;

      if (!IsBlockOnPlane(block, n_inliers))
      {
        contour_blocks_.push_back(block);
        continue;
      }

      n_inliers_total += n_inliers;

      // Add neighboring blocks if they have not been processed
      for (int y = std::max(0, block.y - 1); y <= std::min(block.y + 1, mask_.mask_mini().rows - 1); ++y)
        for (int x = std::max(0, block.x - 1); x <= std::min(block.x + 1, mask_.mask_mini().cols - 1); ++x)
        {
          // Do not process the block if it has been or is about to be processed
          if ((mask_.at_mini(y, x) == 255) || (mask_.at_mini(y, x) == refinement_iteration_))
            continue;

          if (mask_.at_mini(y, x) == refinement_iteration_ - 1)
            mask_.at_mini(y, x) = refinement_iteration_;
          else
            mask_.at_mini(y, x) = 255;

          blocks.push_back(cv::Point2i(x, y));
        }
    }
  }

  std::list<cv::Point2i> contour_blocks_;
  float err_;
  const Plane& plane_;
  const cv::Mat& points3d_;
  cv::Mat_<uchar> &overall_mask_;
  PlaneMask& mask_;
  unsigned char refinement_iteration_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Given a plane and inliers, refine the plane
 * @param points3d
 * @param mask
 * @param n_inliers
 * @param plane
 */
void
RefinePlane(const cv::Mat& samples, Plane& plane)
{
  // Compute the new plane equation
  if (samples.rows < 10)
    return;

  cv::Mat_<float> abcd;
  cv::SVD::solveZ(samples, abcd);
  cv::Mat_<float> abcd3 = (cv::Mat_<float>(3, 1) << abcd(0, 0), abcd(1, 0), abcd(2, 0));
  abcd = abcd / cv::norm(abcd3);
  plane.abcd_[0] = abcd(0, 0);
  plane.abcd_[1] = abcd(1, 0);
  plane.abcd_[2] = abcd(2, 0);
  plane.abcd_[3] = abcd(3, 0);
}

/** Given a plane and inliers, refine the plane
 * @param points3d
 * @param mask
 * @param n_inliers
 * @param plane
 */
void
RefinePlane(const cv::Mat& points3d, const PlaneMask& mask, int n_inliers, Plane& plane)
{
  static cv::RNG rng;
// sample some points
  size_t n_samples = std::max(3, std::min(200, n_inliers / 10));
  size_t n_trials = 2 * n_samples;
  size_t i_trials = 0, i_samples = 0;

  cv::Mat_<float> samples = cv::Mat_<float>::ones(n_samples, 4);
  cv::Range range_x = mask.range_x(), range_y = mask.range_y();
  cv::MatIterator_<float> samples_ptr = samples.begin();

  while ((i_samples < n_samples) && (i_trials < n_trials))
  {
    ++i_trials;
    int x = rng.uniform(range_x.start, range_x.end);
    int y = rng.uniform(range_y.start, range_y.end);

    // Keep the coords that are in the mask
    if (!mask(y, x))
      continue;

    const cv::Vec3f point = points3d.at<cv::Vec3f>(y, x);
    *(samples_ptr++) = point[0];
    *(samples_ptr++) = point[1];
    *(samples_ptr++) = point[2];
    ++samples_ptr;
    ++i_samples;
  }

// Compute the new plane equation
  RefinePlane(samples.rowRange(0, i_samples), plane);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace cv
{
  RgbdPlane::RgbdPlane(int rows, int cols, int depth, const cv::Mat & K, int window_size, RGBD_PLANE_METHOD method)
  {
    rgbd_normals_ = RgbdNormals(rows, cols, depth, K, window_size, RgbdNormals::RGBD_NORMALS_METHOD_FALS);
  }

  /** Find
   * @param depth image. If it has 3 channels, it is assumed to be 2d points
   * @param mask An image where each pixel is labeled with the plane it belongs to
   */
  void
  RgbdPlane::operator()(const cv::Mat & points3d_in, cv::Mat &mask_out, std::vector<cv::Vec4f> & plane_coefficients)
  {
    cv::Mat normals = rgbd_normals_(points3d_in);
    this->operator()(points3d_in, normals, mask_out, plane_coefficients);
  }

  void
  RgbdPlane::operator()(const cv::Mat & points3d_in, const cv::Mat & normals, cv::Mat &mask_out,
                        std::vector<cv::Vec4f> & plane_coefficients)
  {
    // Size of a block to check if it belongs to a plane (in pixels)
    size_t block_size_ = 40;
    // Number of inliers to consider to define a plane.
    size_t n_inliers_ = 50;
    // Number of samples to draw to check if we have a plane.
    size_t n_samples_ = 300;
    // Number of trials to make to find a plane.
    size_t n_trials_ = 100;
    // Error (in meters) for how far a point is on a plane.
    double error_ = 0.02;

    cv::Mat_<cv::Vec3f> points3d_;
    points3d_in.convertTo(points3d_, CV_32F);
    /** Output planes */
    std::vector<cv::Vec4f> planes_;
    /** Output mask of the planes */
    std::vector<cv::Mat> masks_;
    /** Output bouding rectangles of the masks of the planes */
    std::vector<cv::Rect> rects_;

    /** An output image that contains dense clusters */
    cv::Mat image_clusters_;

    cv::TickMeter tm;
    tm.start();
    //CALLGRIND_START_INSTRUMENTATION;

    // Pre-computations
    cv::Mat_<uchar> overall_mask = cv::Mat_<uchar>::zeros(points3d_.rows, points3d_.cols);
    std::vector<PlaneMask> masks;

    // Line 3-4
    size_t index_plane = 1;

    std::vector<cv::Point2i> d(3);
    std::vector<cv::Point3f> p(3);
    std::vector<cv::Vec4f> P_hat;
    std::vector<Plane> planes;

    plane_coefficients.clear();
    mask_out.create(points3d_in.rows, points3d_in.cols, CV_8U);
    for (size_t i_trial = 0; i_trial < n_trials_; ++i_trial)
    {
      // Line 5-8
      Sampler sampler(points3d_, overall_mask);

      if (!sampler.Find(d))
        break;

      // Line 9
      P_hat.clear();

      for (unsigned char i = 0; i < 3; ++i)
      {
        cv::Vec4f p;
        const cv::Point3f& p3 = points3d_.at<cv::Point3f>(d[i].y, d[i].x);
        p[0] = p3.x;
        p[1] = p3.y;
        p[2] = p3.z;
        p[3] = 1;
        P_hat.push_back(p);
      }

      // Construct the plane for those 3 points
      Plane plane(P_hat, index_plane);

      // Define the boundaries for sampling
      size_t numInliers = 0;

      int x_min = std::min(std::min(d[0].x, d[1].x), d[2].x);
      int x_max = std::max(std::max(d[0].x, d[1].x), d[2].x);
      int y_min = std::min(std::min(d[0].y, d[1].y), d[2].y);
      int y_max = std::max(std::max(d[0].y, d[1].y), d[2].y);
      // nu could be yet another parameter
      int nu = block_size_;
      x_min = std::max(0, x_min - nu);
      x_max = std::min(overall_mask.cols, x_max + nu + 1);
      y_min = std::max(0, y_min - nu);
      y_max = std::min(overall_mask.rows, y_max + nu + 1);

      for (size_t i_sample = 3; i_sample < n_samples_; ++i_sample)
      {
        // Create a sample
        cv::Point2i d_j;
        static cv::RNG rng;
        d_j.y = rng.uniform(y_min, y_max);
        d_j.x = rng.uniform(x_min, x_max);

        // Make sure it was not used before and that its point is valid
        if (overall_mask(d_j.y, d_j.x))
          continue;

        cv::Point3f p_j = points3d_.at<cv::Point3f>(d_j.y, d_j.x);

        if (cvIsNaN(p_j.x))
          continue;

        if (d_j.x - nu < x_min)
          x_min = std::max(0, d_j.x - nu);
        else if (d_j.x + nu > x_max)
          x_max = std::min(overall_mask.cols, d_j.x + nu);

        if (d_j.y - nu < y_min)
          y_min = std::max(0, d_j.y - nu);
        else if (d_j.y + nu > y_max)
          y_max = std::min(overall_mask.rows, d_j.y + nu);

        // If the point is on the plane
        if (plane.distance(p_j) < error_)
        {
          cv::Vec4f p_j4;
          p_j4[0] = p_j.x;
          p_j4[1] = p_j.y;
          p_j4[2] = p_j.z;
          p_j4[3] = 1;
          P_hat.push_back(p_j4);
          ++numInliers;
        }
      }

      if (!(numInliers > n_inliers_))
        continue;

      // Refine the plane a bit
      {
        cv::Mat samples = cv::Mat(P_hat);
        samples = samples.reshape(1, P_hat.size());
        RefinePlane(samples, plane);
      }

      // Define the mask structure used during refinement
      PlaneMask mask(points3d_.rows, points3d_.cols, block_size_);
      masks.push_back(mask);

      int n_inliers = 0;
      // std::cout << "*************** " << i << std::endl;
      // std::cout << plane << std::endl;
      // Two passes of refinement are usually enough (we could have an error based criterion)
      InlierFinder inlier_finder(error_, plane, points3d_, overall_mask, mask);

      for (unsigned char i = 0; i < 2; ++i)
      {
        if (i == 0)
          // Process blocks starting at d[0]
          inlier_finder.Find(d[0], n_inliers);
        else
          inlier_finder.Find(n_inliers);

        // std::cout << "n inliers " << n_inliers << std::endl;

        RefinePlane(points3d_, mask, n_inliers, plane);
        // std::cout << plane << std::endl;
      }
      mask_out.setTo(index_plane, mask.mask());

      ++index_plane;
      planes.push_back(plane);
      plane_coefficients.push_back(plane.abcd_);
    };

    //CALLGRIND_STOP_INSTRUMENTATION;
    tm.stop();

    std::cout << "Time = " << tm.getTimeMilli() << " msec." << std::endl;
  }
}
