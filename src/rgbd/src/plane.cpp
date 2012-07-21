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

#include <boost/foreach.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/rgbd/rgbd.hpp>

/** Structure defining a plane */
class Plane
{
public:
  Plane(const cv::Vec3f & m, const cv::Vec3f &n, int index)
      :
        index_(index),
        m_(m),
        Q_(cv::Matx33f::zeros()),
        n_(n),
        mse_(0),
        K_(0)
  {
    UpdateAbcd();
  }

  inline
  float
  distance(const cv::Vec3f& p_j) const
  {
    return std::abs(abcd_[0] * p_j[0] + abcd_[1] * p_j[1] + abcd_[2] * p_j[2] + abcd_[3]);
  }

  const cv::Vec4f &
  abcd() const
  {
    return abcd_;
  }

  void
  UpdateParameters()
  {
    if (empty())
      return;
    m_ = m_sum_ / K_;
    // Compute C
    cv::Matx33f C = Q_ - 2 * K_ * m_ * m_.t() + K_ * m_ * m_.t();

    // Compute n
    cv::SVD svd(C);
    n_ = cv::Vec3f(svd.vt.at<float>(2, 0), svd.vt.at<float>(2, 1), svd.vt.at<float>(2, 2));
    d_ = n_.dot(m_);
    mse_ = svd.w.at<float>(2) / K_;

    UpdateAbcd();
  }

  void
  UpdateStatistics(const cv::Vec3f & point)
  {
    m_sum_ += point;
    Q_ += point * point.t();
    ++K_;
  }

  inline size_t
  empty() const
  {
    return K_ == 0;
  }
  /** The index of the plane */
  int index_;
private:
  inline void
  UpdateAbcd()
  {
    abcd_ = cv::Vec4f(n_[0], n_[1], n_[2], -m_.dot(n_));
  }
  /** coefficients for ax+by+cz+d = 0 */
  cv::Vec4f abcd_;
  /** The sum of the points */
  cv::Vec3f m_sum_;
  /** The mean of the points */
  cv::Vec3f m_;
  /** The sum of pi * pi^\top */
  cv::Matx33f Q_;
  /** The different matrices we need to update */
  cv::Matx33f C_;
  cv::Vec3f n_, d_;
  float mse_;
  /** the number of points that form the plane */
  int K_;
};

std::ostream&
operator<<(std::ostream& out, const Plane& plane)
{
  out << "coeff: " << plane.abcd()[0] << "," << plane.abcd()[1] << "," << plane.abcd()[2] << "," << plane.abcd()[3]
      << ",";
  return out;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class PlaneGrid
{
public:
  PlaneGrid(cv::Mat_<cv::Vec3f> points3d, int block_size)
      :
        block_size_(block_size)
  {
    // Figure out some dimensions
    int mini_rows = points3d.rows / block_size;
    if (points3d.rows % block_size != 0)
      ++mini_rows;

    int mini_cols = points3d.cols / block_size;
    if (points3d.cols % block_size != 0)
      ++mini_cols;

    // Compute all the interesting quantities
    Q_.create(mini_rows, mini_cols);
    m_.create(mini_rows, mini_cols);
    n_.create(mini_rows, mini_cols);
    K_.create(mini_rows, mini_cols);
    d_.create(mini_rows, mini_cols);
    mse_.create(mini_rows, mini_cols);
    for (int y = 0; y < mini_rows; ++y)
      for (int x = 0; x < mini_cols; ++x)
      {
        // Update the tiles
        cv::Matx33f Q = cv::Matx33f::zeros();
        cv::Vec3f m = cv::Vec3f(0, 0, 0);
        int K = 0;
        for (int j = y * block_size; j < std::min((y + 1) * block_size, points3d.rows); ++j)
        {
          const cv::Vec3f * vec = points3d.ptr<cv::Vec3f>(j, x * block_size), *vec_end;
          if (x == mini_cols)
            vec_end = vec + points3d.cols - 1 - x * block_size;
          else
            vec_end = vec + block_size;
          for (; vec != vec_end; ++vec)
          {
            if (cvIsNaN(vec->val[0]))
              continue;
            Q += (*vec) * (*vec).t();
            m += (*vec);
            ++K;
          }
        }
        if (K == 0)
          continue;

        *(reinterpret_cast<cv::Matx33f *>(Q_.ptr(y, x))) = Q;
        K_(y, x) = K;
        m /= K;
        m_(y, x) = m;

        // Compute C
        cv::Matx33f C = Q - 2 * K * m * m.t() + K * m * m.t();

        // Compute n
        cv::SVD svd(C);
        n_(y, x) = cv::Vec3f(svd.vt.at<float>(2, 0), svd.vt.at<float>(2, 1), svd.vt.at<float>(2, 2));
        d_(y, x) = n_(y, x).dot(m);
        mse_(y, x) = svd.w.at<float>(2) / K;
      }
  }

  /** The size of the block */
  int block_size_;
  cv::Mat_<cv::Vec<float, 9> > Q_;
  cv::Mat_<cv::Vec3f> m_;
  cv::Mat_<cv::Vec3f> n_;
  cv::Mat_<int> K_;
  cv::Mat_<float> d_;
  cv::Mat_<float> mse_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class TileQueue
{
public:
  struct PlaneTile
  {
    PlaneTile(int x, int y, float mse)
        :
          x_(x),
          y_(y),
          mse_(mse)
    {
    }

    bool
    operator<(const PlaneTile &tile2) const
    {
      return mse_ < tile2.mse_;
    }

    int x_;
    int y_;
    float mse_;
  };

  TileQueue(const PlaneGrid &plane_grid)
  {
    done_tiles_ = cv::Mat_<unsigned char>::zeros(plane_grid.mse_.rows, plane_grid.mse_.cols);
    tiles_.clear();
    for (int y = 0; y < plane_grid.mse_.rows; ++y)
      for (int x = 0; x < plane_grid.mse_.cols; ++x)
      {
        // Update the tiles
        tiles_.push_back(PlaneTile(x, y, plane_grid.mse_(y, x)));
      }
    // Sort tiles by MSE
    tiles_.sort();
  }

  bool
  Empty()
  {
    while (!tiles_.empty())
    {
      const PlaneTile & tile = tiles_.front();
      if (done_tiles_(tile.y_, tile.x_))
        tiles_.pop_front();
      else
        break;
    }
    return tiles_.empty();
  }

  const PlaneTile &
  front() const
  {
    return tiles_.front();
  }

  size_t
  size() const
  {
    return tiles_.size();
  }

  void
  remove(int y, int x)
  {
    done_tiles_(y, x) = 1;
  }
private:
  /** The list of tiles ordered from most planar to least */
  std::list<PlaneTile> tiles_;
  /** contains 1 when the tiles has been studied, 0 otherwise */
  cv::Mat_<unsigned char> done_tiles_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** A structure that contains the mask of a plane, as well as extra info */
class PlaneMask
{
public:
  PlaneMask(int rows, int cols, int block_size)
      :
        block_size_(block_size)
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

  const cv::Mat_<uchar>&
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

  uchar&
  at_mini(int y, int x)
  {
    return mask_mini_(y, x);
  }

  void
  set(int y, int x)
  {
    mask_mini_(y, x) = 1;
  }

private:
  /** The size of the block */
  int block_size_;
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class InlierFinder
{
public:
  InlierFinder(float err, const cv::Mat& points3d, unsigned char plane_index)
      :
        err_(err),
        points3d_(points3d),
        plane_index_(plane_index)
  {
  }

  void
  Find(const PlaneGrid &plane_grid, Plane & plane, TileQueue & tile_queue,
       std::set<TileQueue::PlaneTile> & neighboring_tiles, cv::Mat_<unsigned char> & overall_mask,
       PlaneMask & plane_mask)
  {
    // Do not use reference as we pop the from later on
    TileQueue::PlaneTile tile = *(neighboring_tiles.begin());

    // Figure the part of the image to look at
    cv::Mat point3d_reshape;
    cv::Range range_x, range_y;
    int x = tile.x_ * plane_mask.block_size(), y = tile.y_ * plane_mask.block_size();

    if (tile.x_ == plane_mask.mask_mini().cols - 1)
      range_x = cv::Range(x, overall_mask.cols);
    else
      range_x = cv::Range(x, x + plane_mask.block_size());

    if (tile.y_ == plane_mask.mask_mini().rows - 1)
      range_y = cv::Range(y, overall_mask.rows);
    else
      range_y = cv::Range(y, y + plane_mask.block_size());

    int n_valid_points = 0;
    bool do_left = false, do_right = false, do_top = false, do_bottom = false;
    for (int yy = range_y.start; yy != range_y.end; ++yy)
    {
      uchar* data = overall_mask.ptr(yy, range_x.start), *data_end = overall_mask.ptr(yy, range_x.end);
      const cv::Vec3f* point = points3d_.ptr<cv::Vec3f>(yy, range_x.start);

      for (int xx = range_x.start; data != data_end; ++data, ++point, ++xx)
      {
        // Don't do anything if the point already belongs to another plane
        if (cvIsNaN(point->val[0]) || ((*data) != 255))
          continue;

        // If the point is close enough to the plane
        if (plane.distance(*point) < err_)
        {
          // TODO make sure the normals are similar to the plane
          if (true)
          {
            // The point now belongs to the plane
            plane.UpdateStatistics(*point);
            *data = plane_index_;
            ++n_valid_points;
            if (yy == range_y.start)
              do_top = true;
            if (yy == range_y.end - 1)
              do_bottom = true;
            if (xx == range_x.start)
              do_left = true;
            if (xx == range_x.end - 1)
              do_right = true;
          }
        }
      }
    }

    plane.UpdateParameters();

    // Mark the front as being done and pop it
    if (n_valid_points > range_x.size() * range_y.size())
      tile_queue.remove(tile.y_, tile.x_);
    plane_mask.set(tile.y_, tile.x_);
    neighboring_tiles.erase(neighboring_tiles.begin());

    // Add potential neighbors of the tile
    std::vector<std::pair<int, int> > pairs;
    if (do_left && tile.x_ > 0)
      pairs.push_back(std::pair<int, int>(tile.x_ - 1, tile.y_));
    if (do_right && tile.x_ < plane_mask.mask_mini().cols - 1)
      pairs.push_back(std::pair<int, int>(tile.x_ + 1, tile.y_));
    if (do_top && tile.y_ > 0)
      pairs.push_back(std::pair<int, int>(tile.x_, tile.y_ - 1));
    if (do_bottom && tile.y_ < plane_mask.mask_mini().rows - 1)
      pairs.push_back(std::pair<int, int>(tile.x_, tile.y_ + 1));

    for (unsigned char i = 0; i < pairs.size(); ++i)
      if (!plane_mask.mask_mini()(pairs[i].second, pairs[i].first))
        neighboring_tiles.insert(
            TileQueue::PlaneTile(pairs[i].first, pairs[i].second, plane_grid.mse_(pairs[i].second, pairs[i].first)));
  }

private:
  float err_;
  const cv::Mat& points3d_;
  unsigned char plane_index_;
}
;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace cv
{
  void
  RgbdPlane::operator()(const cv::Mat & points3d_in, const cv::Mat & normals, cv::Mat &mask_out,
                        std::vector<cv::Vec4f> & plane_coefficients)
  {
    // Size of a block to check if it belongs to a plane (in pixels)
    size_t block_size = 40;
    // Error (in meters) for how far a point is on a plane.
    double error_ = 0.02;

    cv::Mat_<cv::Vec3f> points3d_;
    if (points3d_in.depth() == CV_32F)
      points3d_ = points3d_in;
    else
      points3d_in.convertTo(points3d_, CV_32F);

    // Pre-computations
    cv::Mat_<uchar> overall_mask = cv::Mat_<uchar>(points3d_.rows, points3d_.cols, (unsigned char) (255));

    PlaneGrid plane_grid(points3d_, block_size);
    TileQueue plane_queue(plane_grid);

    // Line 3-4
    size_t index_plane = 0;

    std::vector<Plane> planes;

    plane_coefficients.clear();
    mask_out.create(points3d_in.rows, points3d_in.cols, CV_8U);
    float mse_min = 0.0001;

    while (!plane_queue.Empty())
    {
      // Get the first tile if it's good enough
      const TileQueue::PlaneTile front_tile = plane_queue.front();
      if (front_tile.mse_ > mse_min)
        break;

      InlierFinder inlier_finder(error_, points3d_, index_plane);

      // Construct the plane for those 3 points
      int x = front_tile.x_, y = front_tile.y_;
      const cv::Vec3f & n = plane_grid.n_(y, x);
      Plane plane(plane_grid.m_(y, x), n, index_plane);

      PlaneMask plane_mask(points3d_.rows, points3d_.cols, block_size);
      std::set<TileQueue::PlaneTile> neighboring_tiles;
      neighboring_tiles.insert(front_tile);
      plane_queue.remove(front_tile.y_, front_tile.x_);

      // Process all the neighboring tiles
      while (!neighboring_tiles.empty())
        inlier_finder.Find(plane_grid, plane, plane_queue, neighboring_tiles, overall_mask, plane_mask);

      if (plane.empty())
        continue;

      ++index_plane;
      planes.push_back(plane);
      plane_coefficients.push_back(plane.abcd());
    };
    overall_mask.copyTo(mask_out);
  }
}
