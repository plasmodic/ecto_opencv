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

#include <iostream>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/rgbd/rgbd.hpp>

namespace
{
  template<typename T>
  T
  inline
  norm_vec(const cv::Vec<T, 3> &vec)
  {
    return std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
  }

  /** Given 3d points, compute their distance to the origin
   * @param points
   * @return
   */
  template<typename T>
  cv::Mat_<T>
  computeR(const cv::Mat &points)
  {
    typedef cv::Vec<T, 3> PointT;

    // Compute the
    cv::Size size(points.cols, points.rows);
    cv::Mat_<T> r = cv::Mat_<T>(size);
    if (points.isContinuous())
      size = cv::Size(points.cols * points.rows, 1);
    for (int y = 0; y < size.height; ++y)
    {
      const PointT* point = points.ptr<PointT>(y), *point_end = points.ptr<PointT>(y) + size.width;
      T * row = r[y];
      for (; point != point_end; ++point, ++row)
        *row = norm_vec(*point);
    }

    return r;
  }

  // Compute theta and phi according to equation 3
  template<typename T>
  void
  computeThetaPhi(int rows, int cols, const cv::Matx<T, 3, 3>& K, cv::Mat &cos_theta, cv::Mat &sin_theta,
                  cv::Mat &cos_phi, cv::Mat &sin_phi)
  {
    // Create some bogus coordinates
    cv::Mat depth_image = K(0, 0) * cv::Mat_<T>::ones(rows, cols);
    cv::Mat points3d;
    depthTo3d(depth_image, cv::Mat(K), points3d);

    typedef cv::Matx<T, 3, 1> Vec_T;

    cos_theta = cv::Mat_<T>(rows, cols);
    sin_theta = cv::Mat_<T>(rows, cols);
    cos_phi = cv::Mat_<T>(rows, cols);
    sin_phi = cv::Mat_<T>(rows, cols);
    cv::Mat r = computeR<T>(points3d);
    for (int y = 0; y < rows; ++y)
    {
      T *row_cos_theta = cos_theta.ptr<T>(y), *row_sin_theta = sin_theta.ptr<T>(y);
      T *row_cos_phi = cos_phi.ptr<T>(y), *row_sin_phi = sin_phi.ptr<T>(y);
      const Vec_T * row_points = points3d.ptr<Vec_T>(y), *row_points_end = points3d.ptr<Vec_T>(y) + points3d.cols;
      const T * row_r = r.ptr<T>(y);
      for (; row_points < row_points_end;
          ++row_cos_theta, ++row_sin_theta, ++row_cos_phi, ++row_sin_phi, ++row_points, ++row_r)
      {
        // In the paper, z goes away from the camera, y goes down, x goes right
        // OpenCV has the same conventions
        // Theta goes from z to x (and actually goes from -pi/2 to pi/2, phi goes from z to y
        float theta = std::atan2(row_points->val[0], row_points->val[2]);
        *row_cos_theta = std::cos(theta);
        *row_sin_theta = std::sin(theta);
        float phi = std::asin(row_points->val[1] / (*row_r));
        *row_cos_phi = std::cos(phi);
        *row_sin_phi = std::sin(phi);
      }
    }
  }

  /** Modify normals to make sure they point towards the camera
   * @param normals
   */
  template<typename T>
  void
  signNormals(cv::Mat & normals)
  {
    // Check whether we can process the image as a big row.
    cv::Size size(normals.cols, normals.rows);
    if (normals.isContinuous())
      size = cv::Size(size.width * size.height, 1);

    for (int y = 0; y < size.height; ++y)
    {
      T* row = normals.ptr<T>(y), *row_end = normals.ptr<T>(y) + size.width;
      for (; row != row_end; ++row)
        if ((*row)[2] > 0)
          *row = -(*row) / norm_vec(*row);
        else
          *row = (*row) / norm_vec(*row);
    }
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace
{
  template<typename T>
  class FALS: public cv::RgbdNormals::RgbdNormalsImpl
  {
  public:
    typedef cv::Matx<T, 3, 3> Mat33T;
    typedef cv::Vec<T, 3> PointT;

    FALS(int rows, int cols, int window_size, const cv::Matx<T, 3, 3> &K)
        :
          rows_(rows),
          cols_(cols),
          window_size_(window_size),
          K_(K)
    {
    }
    ~FALS()
    {
    }

    /** Compute cached data
     */
    virtual void
    cache()
    {
      // Compute theta and phi according to equation 3
      cv::Mat cos_theta, sin_theta, cos_phi, sin_phi;
      computeThetaPhi<T>(rows_, cols_, K_, cos_theta, sin_theta, cos_phi, sin_phi);

      // Compute all the v_i for every points
      std::vector<cv::Mat> channels(3);
      channels[0] = sin_theta.mul(cos_phi);
      channels[1] = sin_phi;
      channels[2] = cos_theta.mul(cos_phi);
      cv::Mat V;
      cv::merge(channels, V);
      V_ = V;

      // Compute M and its inverse
      cv::Mat_<cv::Vec<T, 9> > M(rows_, cols_);
      Mat33T VVt;
      for (int y = 0; y < rows_; ++y)
      {
        for (int x = 0; x < cols_; ++x)
        {
          PointT vec(channels[0].at<T>(y, x), channels[1].at<T>(y, x), channels[2].at<T>(y, x));
          VVt = vec * vec.t();
          M(y, x) = cv::Vec<T, 9>(VVt.val);
        }
      }

      cv::boxFilter(M, M, M.depth(), cv::Size(window_size_, window_size_), cv::Point(-1, -1), false);

      Mat33T M_inv;
      M_inv_ = cv::Mat_<cv::Vec<T, 9> >(rows_, cols_);
      for (int y = 0; y < rows_; ++y)
        for (int x = 0; x < cols_; ++x)
        {
          // We have a semi-definite matrix
          cv::invert(Mat33T(M(y, x).val), M_inv, cv::DECOMP_CHOLESKY);
          M_inv_(y, x) = cv::Vec<T, 9>(M_inv.val);
        }
    }

    /** Compute the normals
     * @param r
     * @return
     */
    virtual cv::Mat
    compute(const cv::Mat &r) const
    {
      cv::TickMeter tm1, tm2, tm3;
      tm1.start();
      // Compute B
      cv::Mat_<PointT> B = cv::Mat_<PointT>(rows_, cols_);

      cv::Size size(cols_, rows_);
      if (B.isContinuous() && V_.isContinuous() && r.isContinuous())
        size = cv::Size(cols_ * rows_, 1);

      for (int y = 0; y < size.height; ++y)
      {
        const T* row_r = r.ptr<T>(y), *row_r_end = row_r + size.width;
        const PointT *row_V = V_[y];
        PointT *row_B = B[y];
        for (; row_r != row_r_end; ++row_r, ++row_B, ++row_V)
        {
          if (cvIsNaN(*row_r))
            *row_B = PointT();
          else
            *row_B = (*row_V) / (*row_r);
        }
      }
      tm1.stop();

      // Apply a box filter to B
      tm2.start();
      cv::boxFilter(B, B, B.depth(), cv::Size(window_size_, window_size_), cv::Point(-1, -1), false);
      tm2.stop();

      // compute the Minv*B products
      tm3.start();
      cv::Mat_<PointT> normals(rows_, cols_);
      size = cv::Size(cols_, rows_);
      if (B.isContinuous() && M_inv_.isContinuous() && normals.isContinuous())
        size = cv::Size(cols_ * rows_, 1);
      for (int y = 0; y < size.height; ++y)
      {
        const PointT * B_vec = B[y], *B_vec_end = B_vec + size.width;
        const Mat33T * M_inv = reinterpret_cast<const Mat33T *>(M_inv_.ptr(y));
        PointT *normal = normals[y];
        for (; B_vec != B_vec_end; ++B_vec, ++normal, ++M_inv)
          *normal = (*M_inv) * (*B_vec);
      }
      tm3.stop();

      std::cout << tm1.getTimeMilli() << " " << tm2.getTimeMilli() << " " << tm3.getTimeMilli() << " ";

      return normals;
    }
  private:
    int rows_;
    int cols_;
    int window_size_;
    Mat33T K_;

    cv::Mat_<PointT> V_;
    cv::Mat_<cv::Vec<T, 9> > M_inv_;
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace
{
  template<typename T>
  class SRI: public cv::RgbdNormals::RgbdNormalsImpl
  {
  public:
    typedef cv::Matx<T, 3, 3> Mat33T;
    typedef cv::Vec<T, 3> PointT;

    SRI(int rows, int cols, int window_size, const cv::Matx<T, 3, 3> &K)
        :
          rows_(rows),
          cols_(cols),
          window_size_(window_size),
          K_(K)
    {
    }

    /** Compute cached data
     */
    virtual void
    cache()
    {
      cv::Mat_<T> cos_theta, sin_theta, cos_phi, sin_phi;
      computeThetaPhi<T>(rows_, cols_, K_, cos_theta, sin_theta, cos_phi, sin_phi);

      R_hat_.create(rows_, cols_);
      for (int y = 0; y < rows_; ++y)
        for (int x = 0; x < cols_; ++x)
        {
          cv::Mat_<T> mat =
              (cv::Mat_<T>(3, 3) << 0, 1, 0, 0, 0, 1, 1, 0, 0)
              * ((cv::Mat_<T>(3, 3) << cos_theta(y, x), -sin_theta(y, x), 0, sin_theta(y, x), cos_theta(y, x), 0, 0, 0, 1))
              * ((cv::Mat_<T>(3, 3) << cos_phi(y, x), 0, -sin_phi(y, x), 0, 1, 0, sin_phi(y, x), 0, cos_phi(y, x)));
          for (unsigned char i = 0; i < 3; ++i)
            mat(i, 1) = mat(i, 1) / cos_phi(y, x);

          cv::Vec<T, 9> &vec = R_hat_(y, x);
          vec = cv::Vec<T, 9>((T*) (mat.data));
        }
    }

    /** Compute the normals
     * @param r
     * @return
     */
    virtual cv::Mat
    compute(const cv::Mat &r) const
    {
      const cv::Mat_<T>& r_T(r);
      return compute(r_T);
    }

    /** Compute the normals
     * @param r
     * @return
     */
    cv::Mat
    compute(const cv::Mat_<T> &r) const
    {
      cv::TickMeter tm1, tm2;

      // Compute the derivatives with respect to theta and phi
      tm1.start();
      cv::Mat_<T> r_theta, r_phi;
      cv::Sobel(r, r_theta, r.depth(), 1, 0, window_size_);
      cv::Sobel(r, r_phi, r.depth(), 0, 1, window_size_);
      tm1.stop();

      // Fill the result matrix
      tm2.start();
      cv::Mat_<PointT> normals(rows_, cols_);
      cv::Size size(cols_, rows_);
      if (r_theta.isContinuous() && r_phi.isContinuous() && R_hat_.isContinuous() && r.isContinuous()
          && normals.isContinuous())
        size = cv::Size(rows_ * cols_, 1);

      for (int y = 0; y < size.height; ++y)
      {
        const T* r_theta_ptr = r_theta[y], *r_theta_ptr_end = r_theta_ptr + size.width;
        const T* r_phi_ptr = r_phi[y];
        const cv::Matx<T, 3, 3> * R = reinterpret_cast<const cv::Matx<T, 3, 3> *>(R_hat_.ptr(y));
        const T* r_ptr = r[y];
        PointT * normal = normals[y];
        for (; r_theta_ptr != r_theta_ptr_end; ++r_theta_ptr, ++r_phi_ptr, ++R, ++r_ptr, ++normal)
        {
          T r_theta_over_r = (*r_theta_ptr) / (*r_ptr);
          T r_phi_over_r = (*r_phi_ptr) / (*r_ptr);
          (*normal)[0] = (*R)(0, 0) + (*R)(0, 1) * r_theta_over_r + (*R)(0, 2) * r_phi_over_r;
          (*normal)[1] = (*R)(1, 0) + (*R)(1, 1) * r_theta_over_r + (*R)(1, 2) * r_phi_over_r;
          (*normal)[2] = (*R)(2, 0) + (*R)(2, 1) * r_theta_over_r + (*R)(2, 2) * r_phi_over_r;
        }
      }
      tm2.stop();

      std::cout << tm1.getTimeMilli() << " " << tm2.getTimeMilli() << " ";
      return normals;
    }
  private:
    int rows_;
    int cols_;
    int window_size_;
    cv::Matx<T, 3, 3> K_;

    /** Stores R */
    cv::Mat_<cv::Vec<T, 9> > R_hat_;
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace cv
{
  /** Default constructor
   */
  RgbdNormals::RgbdNormals(int rows, int cols, int depth, const cv::Mat & K, RGBD_NORMALS_METHOD method)
      :
        method_(method)
  {
    CV_Assert(depth == CV_32F || depth == CV_64F);

    cv::Mat K_right_depth;
    K.convertTo(K_right_depth, depth);
    K_ = K_right_depth;

    // TODO make it a parameter
    int window_size = 5;
    switch (method_)
    {
      case RGBD_NORMALS_METHOD_SRI:
      {
        if (depth == CV_32F)
          rgbd_normals_impl_ = new SRI<float>(rows, cols, window_size, K_right_depth);
        else
          rgbd_normals_impl_ = new SRI<double>(rows, cols, window_size, K_right_depth);
        break;
      }
      case (RGBD_NORMALS_METHOD_FALS):
      {
        if (depth == CV_32F)
          rgbd_normals_impl_ = new FALS<float>(rows, cols, window_size, K_right_depth);
        else
          rgbd_normals_impl_ = new FALS<double>(rows, cols, window_size, K_right_depth);
        break;
      }
    }
    rgbd_normals_impl_->cache();
  }

  /** Given a set of 3d points in a depth image, compute the normals at each point
   * using the SRI method described in
   * ``Fast and Accurate Computation of Surface Normals from Range Images``
   * by H. Badino, D. Huber, Y. Park and T. Kanade
   * @param depth depth a float depth image. Or it can be rows x cols x 3 is they are 3d points
   * @param window_size the window size on which to compute the derivatives
   * @return normals a rows x cols x 3 matrix
   */
  cv::Mat
  RgbdNormals::operator()(const cv::Mat &in_points3d, int window_size) const
  {
    cv::TickMeter tm1, tm2, tm_all;
    CV_Assert(in_points3d.channels() == 3 && in_points3d.dims == 2);
    CV_Assert(in_points3d.depth() == CV_32F || in_points3d.depth() == CV_64F);

    // Make the points have the right depth
    tm_all.start();
    cv::Mat points3d;
    in_points3d.convertTo(points3d, K_.depth());

    // Compute the distance to the points
    tm1.start();
    cv::Mat r;
    if (K_.depth() == CV_32F)
      r = computeR<float>(points3d);
    else
      r = computeR<double>(points3d);
    tm1.stop();
    std::cout << "Time = " << tm1.getTimeMilli() << " ";

    // Get the normals
    cv::Mat normals = rgbd_normals_impl_->compute(r);

    // Make sure the normals point towards the camera
    tm2.start();
    if (normals.depth() == CV_32F)
      signNormals<cv::Vec3f>(normals);
    else
      signNormals<cv::Vec3d>(normals);
    tm2.stop();
    tm_all.stop();

    std::cout << tm2.getTimeMilli() << " all: " << tm_all.getTimeMilli() << " msec." << std::endl;

    return normals;
  }
}
