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

#include <opencv2/calib3d/calib3d.hpp>
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
    cv::Mat_<T> r(size);
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

    typedef cv::Vec<T, 3> Vec3T;

    cos_theta = cv::Mat_<T>(rows, cols);
    sin_theta = cv::Mat_<T>(rows, cols);
    cos_phi = cv::Mat_<T>(rows, cols);
    sin_phi = cv::Mat_<T>(rows, cols);
    cv::Mat r = computeR<T>(points3d);
    for (int y = 0; y < rows; ++y)
    {
      T *row_cos_theta = cos_theta.ptr<T>(y), *row_sin_theta = sin_theta.ptr<T>(y);
      T *row_cos_phi = cos_phi.ptr<T>(y), *row_sin_phi = sin_phi.ptr<T>(y);
      const Vec3T * row_points = points3d.ptr<Vec3T>(y), *row_points_end = points3d.ptr<Vec3T>(y) + points3d.cols;
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
  inline
  void
  signNormal(const cv::Vec<T, 3> & normal_in, cv::Vec<T, 3> & normal_out)
  {
    if (normal_in[2] > 0)
      normal_out = -normal_in / norm_vec(normal_in);
    else
      normal_out = normal_in / norm_vec(normal_in);
  }
  /** Modify normals to make sure they point towards the camera
   * @param normals
   */
  template<typename T>
  inline
  void
  signNormal(T a, T b, T c, cv::Vec<T, 3> & normal)
  {
    T norm = 1 / std::sqrt(a * a + b * b + c * c);
    if (c > 0)
    {
      normal[0] = -a * norm;
      normal[1] = -b * norm;
      normal[2] = -c * norm;
    }
    else
    {
      normal[0] = a * norm;
      normal[1] = b * norm;
      normal[2] = c * norm;
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
    typedef cv::Vec<T, 9> Vec9T;
    typedef cv::Vec<T, 3> Vec3T;

    FALS(int rows, int cols, int window_size, const Mat33T &K)
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
      cv::merge(channels, V_);

      // Compute M
      cv::Mat_<Vec9T> M(rows_, cols_);
      Mat33T VVt;
      const Vec3T * vec = V_[0];
      Vec9T * M_ptr = M[0], *M_ptr_end = M_ptr + rows_ * cols_;
      for (; M_ptr != M_ptr_end; ++vec, ++M_ptr)
      {
        VVt = (*vec) * vec->t();
        *M_ptr = Vec9T(VVt.val);
      }

      cv::boxFilter(M, M, M.depth(), cv::Size(window_size_, window_size_), cv::Point(-1, -1), false);

      // Compute M's inverse
      Mat33T M_inv;
      M_inv_.create(rows_, cols_);
      Vec9T * M_inv_ptr = M_inv_[0];
      for (M_ptr = &M(0); M_ptr != M_ptr_end; ++M_inv_ptr, ++M_ptr)
      {
        // We have a semi-definite matrix
        cv::invert(Mat33T(M_ptr->val), M_inv, cv::DECOMP_CHOLESKY);
        *M_inv_ptr = Vec9T(M_inv.val);
      }
    }

    /** Compute the normals
     * @param r
     * @return
     */
    virtual cv::Mat
    compute(const cv::Mat&, const cv::Mat &r) const
    {
      // Compute B
      cv::Mat_<Vec3T> B(rows_, cols_);

      const T* row_r = r.ptr<T>(0), *row_r_end = row_r + rows_ * cols_;
      const Vec3T *row_V = V_[0];
      Vec3T *row_B = B[0];
      for (; row_r != row_r_end; ++row_r, ++row_B, ++row_V)
      {
        if (cvIsNaN(*row_r))
          *row_B = Vec3T();
        else
          *row_B = (*row_V) / (*row_r);
      }

      // Apply a box filter to B
      cv::boxFilter(B, B, B.depth(), cv::Size(window_size_, window_size_), cv::Point(-1, -1), false);

      // compute the Minv*B products
      cv::Mat_<Vec3T> normals(rows_, cols_);
      row_r = r.ptr<T>(0);
      const Vec3T * B_vec = B[0];
      const Mat33T * M_inv = reinterpret_cast<const Mat33T *>(M_inv_.ptr(0));
      Vec3T *normal = normals[0];
      for (; row_r != row_r_end; ++row_r, ++B_vec, ++normal, ++M_inv)
        if (cvIsNaN(*row_r))
        {
          (*normal)[0] = *row_r;
          (*normal)[1] = *row_r;
          (*normal)[2] = *row_r;
        }
        else
          signNormal((*M_inv) * (*B_vec), *normal);

      return normals;
    }
  private:
    int rows_;
    int cols_;
    int window_size_;
    Mat33T K_;

    cv::Mat_<Vec3T> V_;
    cv::Mat_<Vec9T> M_inv_;
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
    typedef cv::Vec<T, 9> Vec9T;
    typedef cv::Vec<T, 3> Vec3T;

    SRI(int rows, int cols, int window_size, const cv::Matx<T, 3, 3> &K)
        :
          rows_(rows),
          cols_(cols),
          window_size_(window_size),
          K_(K),
          phi_step_(0),
          theta_step_(0)
    {
    }

    /** Compute cached data
     */
    virtual void
    cache()
    {
      cv::Mat_<T> cos_theta, sin_theta, cos_phi, sin_phi;
      computeThetaPhi<T>(rows_, cols_, K_, cos_theta, sin_theta, cos_phi, sin_phi);

      // Create the derivative kernels
      getDerivKernels(kx_dx_, ky_dx_, 1, 0, window_size_, true, K_.depth);
      getDerivKernels(kx_dy_, ky_dy_, 0, 1, window_size_, true, K_.depth);

      // Get the mapping function for SRI
      float min_theta = std::asin(sin_theta(0, 0)), max_theta = std::asin(sin_theta(0, cols_ - 1));
      float min_phi = std::asin(sin_phi(0, 0)), max_phi = std::asin(sin_phi(rows_ - 1, 0));

      std::vector<cv::Point3f> points3d(cols_ * rows_);
      R_hat_.create(rows_, cols_);
      phi_step_ = float(max_phi - min_phi) / (rows_ - 1);
      theta_step_ = float(max_theta - min_theta) / (cols_ - 1);
      for (int phi_int = 0, k = 0; phi_int < rows_; ++phi_int)
      {
        float phi = min_phi + phi_int * phi_step_;
        for (int theta_int = 0; theta_int < cols_; ++theta_int, ++k)
        {
          float theta = min_theta + theta_int * theta_step_;
          // Store the 3d point to project it later
          points3d[k] = cv::Point3f(std::sin(theta) * std::cos(phi), std::sin(phi), std::cos(theta) * std::cos(phi));

          // Cache the rotation matrix and negate it
          cv::Mat_<T> mat =
              (cv::Mat_<T>(3, 3) << 0, 1, 0, 0, 0, 1, 1, 0, 0) * ((cv::Mat_<T>(3, 3) << std::cos(theta), -std::sin(
                  theta), 0, std::sin(theta), std::cos(theta), 0, 0, 0, 1))
              * ((cv::Mat_<T>(3, 3) << std::cos(phi), 0, -std::sin(phi), 0, 1, 0, std::sin(phi), 0, std::cos(phi)));
          for (unsigned char i = 0; i < 3; ++i)
            mat(i, 1) = mat(i, 1) / std::cos(phi);
          // The second part of the matrix is never explained in the paper ... but look at the wikipedia normal article
          mat(0, 0) = mat(0, 0) - 2 * std::cos(phi) * std::sin(theta);
          mat(1, 0) = mat(1, 0) - 2 * std::sin(phi);
          mat(2, 0) = mat(2, 0) - 2 * std::cos(phi) * std::cos(theta);

          R_hat_(phi_int, theta_int) = Vec9T((T*) (mat.data));
        }
      }

      map_.create(rows_, cols_);
      cv::Mat rvec;
      cv::Rodrigues(cv::Mat::eye(3, 3, CV_32F), rvec);
      cv::Mat tvec = (cv::Mat_<float>(1, 3) << 0, 0, 0);
      cv::projectPoints(points3d, rvec, tvec, K_, cv::Mat(), map_);
      map_ = map_.reshape(2, rows_);

      // Update the kernels: the steps are dues to the fact that derivatives will be computed on a grid where
      // the step is not 1. Only need to do it on one dimension as it computes derivatives in only one direction
      kx_dx_ /= theta_step_;
      ky_dy_ /= phi_step_;
    }

    /** Compute the normals
     * @param r
     * @return
     */
    virtual cv::Mat
    compute(const cv::Mat& points3d, const cv::Mat &r) const
    {
      const cv::Mat_<T>& r_T(r);
      const cv::Mat_<Vec3T> &points3d_T(points3d);
      return compute(points3d_T, r_T);
    }

    /** Compute the normals
     * @param r
     * @return
     */
    cv::Mat
    compute(const cv::Mat_<Vec3T> &, const cv::Mat_<T> &r_non_interp) const
    {
      // Interpolate the radial image to make derivatives meaningful
      cv::Mat_<T> r;
      // higher quality remapping does not help here
      cv::remap(r_non_interp, r, map_, cv::Mat(), CV_INTER_LINEAR);

      // Compute the derivatives with respect to theta and phi
      // TODO add bilateral filtering (as done in kinfu)
      cv::Mat_<T> r_theta, r_phi;
      cv::sepFilter2D(r, r_theta, r.depth(), kx_dx_, ky_dx_);
      cv::sepFilter2D(r, r_phi, r.depth(), kx_dy_, ky_dy_);

      // Fill the result matrix
      cv::Mat_<Vec3T> normals(rows_, cols_);

      const T* r_theta_ptr = r_theta[0], *r_theta_ptr_end = r_theta_ptr + rows_ * cols_;
      const T* r_phi_ptr = r_phi[0];
      const Mat33T * R = reinterpret_cast<const Mat33T *>(R_hat_[0]);
      const T* r_ptr = r[0];
      Vec3T * normal = normals[0];
      for (; r_theta_ptr != r_theta_ptr_end; ++r_theta_ptr, ++r_phi_ptr, ++R, ++r_ptr, ++normal)
      {
        if (cvIsNaN(*r_ptr))
        {
          (*normal)[0] = *r_ptr;
          (*normal)[1] = *r_ptr;
          (*normal)[2] = *r_ptr;
        }
        else
        {
          T r_theta_over_r = (*r_theta_ptr) / (*r_ptr);
          T r_phi_over_r = (*r_phi_ptr) / (*r_ptr);
          // R(1,1) is 0
          signNormal((*R)(0, 0) + (*R)(0, 1) * r_theta_over_r + (*R)(0, 2) * r_phi_over_r,
                     (*R)(1, 0) + (*R)(1, 2) * r_phi_over_r,
                     (*R)(2, 0) + (*R)(2, 1) * r_theta_over_r + (*R)(2, 2) * r_phi_over_r, *normal);
        }
      }

      return normals;
    }
  private:
    int rows_;
    int cols_;
    int window_size_;
    Mat33T K_;

    /** Stores R */
    cv::Mat_<Vec9T> R_hat_;
    float phi_step_, theta_step_;

    /** Derivative kernels */
    cv::Mat kx_dx_, ky_dx_, kx_dy_, ky_dy_;
    /** mapping function to get an SRI image */
    cv::Mat_<cv::Vec2f> map_;
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace cv
{
  /** Default constructor
   */
  RgbdNormals::RgbdNormals(int rows, int cols, int depth, const cv::Mat & K, int window_size,
                           RGBD_NORMALS_METHOD method)
      :
        window_size_(window_size),
        method_(method)
  {
    CV_Assert(depth == CV_32F || depth == CV_64F);

    {
      cv::Mat K_right_depth;
      K.convertTo(K_right_depth, depth);
      K_ = K_right_depth;
    }

    switch (method_)
    {
      case RGBD_NORMALS_METHOD_SRI:
      {
        if (depth == CV_32F)
          rgbd_normals_impl_ = new SRI<float>(rows, cols, window_size_, K_);
        else
          rgbd_normals_impl_ = new SRI<double>(rows, cols, window_size_, K_);
        break;
      }
      case (RGBD_NORMALS_METHOD_FALS):
      {
        if (depth == CV_32F)
          rgbd_normals_impl_ = new FALS<float>(rows, cols, window_size_, K_);
        else
          rgbd_normals_impl_ = new FALS<double>(rows, cols, window_size_, K_);
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
  RgbdNormals::operator()(const cv::Mat &in_points3d) const
  {
    CV_Assert(in_points3d.channels() == 3 && in_points3d.dims == 2);
    CV_Assert(in_points3d.depth() == CV_32F || in_points3d.depth() == CV_64F);
    CV_Assert(!rgbd_normals_impl_.empty());

    // Make the points have the right depth
    cv::Mat points3d;
    if (in_points3d.depth() == K_.depth())
      points3d = in_points3d;
    else
      in_points3d.convertTo(points3d, K_.depth());

    // Compute the distance to the points
    cv::Mat r;
    if (K_.depth() == CV_32F)
      r = computeR<float>(points3d);
    else
      r = computeR<double>(points3d);

    // Get the normals
    cv::Mat normals = rgbd_normals_impl_->compute(points3d, r);

    return normals;
  }
}
