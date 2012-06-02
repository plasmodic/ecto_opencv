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
  /** Given 3d points, compute their distance to the origin
   * @param points
   * @return
   */
  cv::Mat
  computeR(const cv::Mat &points)
  {
    // Test for speed
    cv::Mat r;
#if 1
    cv::multiply(points, points, r);
    r = r.reshape(1, points.cols * points.rows);
    cv::reduce(r, r, 1, CV_REDUCE_SUM);
    cv::sqrt(r, r);
    r = r.reshape(1, points.rows);
#else
    for (int y = 0; y < points.rows; ++y)
    for (int x = 0; x < points.cols; ++x)
    {
      cv::Vec3f point = points.at<cv::Vec3f>(y, x);
      if (std::abs(r.at<float>(y, x) - cv::norm(point)) / cv::norm(point) > 1e-4)
      std::cout << "r badly computed: " << r.at<float>(y, x) << " for " << cv::norm(point) << ", vec: " << point[0]
      << " " << point[1] << " " << point[2] << std::endl;
    }
#endif

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
    cv::Mat r = computeR(points3d);
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
    for (int y = 0; y < normals.rows; ++y)
    {
      T* row = normals.ptr<T>(y), *row_end = normals.ptr<T>(y) + normals.cols;
      for (; row != row_end; ++row)
      {
        if ((*row)[2] > 0)
          *row = -(*row) / cv::norm(*row);
        else
          *row = (*row) / cv::norm(*row);
      }
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
    FALS(int rows, int cols, int window_size, const cv::Mat &K)
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
      cv::Matx<T, 3, 3> VVt;
      for (int y = 0; y < rows_; ++y)
      {
        for (int x = 0; x < cols_; ++x)
        {
          cv::Vec<T, 3> vec(channels[0].at<T>(y, x), channels[1].at<T>(y, x), channels[2].at<T>(y, x));
          V_(y, x) = vec;
          VVt = vec * vec.t();
          M(y, x) = cv::Vec<T, 9>(VVt.val);
        }
      }

      cv::boxFilter(M, M, M.depth(), cv::Size(window_size_, window_size_), cv::Point(-1, -1), false);

      cv::Matx<T, 3, 3> M_inv;
      M_inv_.resize(3);
      for (unsigned char i = 0; i < 3; ++i)
        M_inv_[i] = cv::Mat_<cv::Vec<T, 3> >(rows_, cols_);
      for (int y = 0; y < rows_; ++y)
        for (int x = 0; x < cols_; ++x)
        {
          // We have a semi-definite matrix
          cv::invert(cv::Matx<T, 3, 3>(M(y, x).val), M_inv);
          M_inv = cv::Matx<T, 3, 3>(M(y, x).val).inv();
          for (unsigned char i = 0; i < 3; ++i)
            M_inv_[i](y, x) = cv::Vec<T, 3>(M_inv(i, 0), M_inv(i, 1), M_inv(i, 2));
        }
    }

    /** Compute the normals
     * @param r
     * @return
     */
    virtual cv::Mat
    compute(const cv::Mat &r) const
    {
      // Compute B
      std::vector<cv::Mat> channels(3);
//TODO test for speed
#if 0
      cv::split(V_, channels);
      for (unsigned char i = 0; i < 3; ++i)
      {
        for (int y = 0; y < rows_; ++y)
        for (int x = 0; x < cols_; ++x)
        if (cvIsNaN(channels[i].at<T>(y, x)))

        cv::Mat channels_ini;
        cv::divide(channels[i], r, channels[i]);
      }
      cv::Mat B;
      cv::merge(channels, B);
#else

      cv::Mat B = cv::Mat_<cv::Vec<T, 3> >(rows_, cols_);
      for (int y = 0; y < rows_; ++y)
      {
        const T* row_r = r.ptr<T>(y), *row_r_end = r.ptr<T>(y) + cols_;
        cv::Vec<T, 3> *row_B = B.ptr<cv::Vec<T, 3> >(y);
        const cv::Vec<T, 3> *row_V = (cv::Vec<T, 3> *) V_.ptr(y);
        for (; row_r != row_r_end; ++row_r, ++row_B, ++row_V)
        {
          if (cvIsNaN(*row_r))
            *row_B = cv::Vec<T, 3>();
          else
            *row_B = (*row_V) / (*row_r);
        }
      }
#endif

      cv::boxFilter(B, B, B.depth(), cv::Size(window_size_, window_size_), cv::Point(-1, -1), false);

      for (unsigned char i = 0; i < 3; ++i)
      {
        cv::Mat product = M_inv_[i].mul(B);
        product = product.reshape(1, cols_ * rows_);
        cv::reduce(product, product, 2, CV_REDUCE_SUM);
        channels[i] = product.reshape(1, rows_);
      }
      cv::Mat normals;
      cv::merge(channels, normals);

      //TODO test for speed
#if 0
            normals = cv::Mat_<cv::Vec<T, 3> >(rows_, cols_);
       for (int y = 0; y < V_.rows; ++y)
       for (int x = 0; x < V_.cols; ++x)
       {

       cv::Matx33d mat;
       for (unsigned int j = 0, k = 0; j < 3; ++j)
       for (unsigned int i = 0; i < 3; ++i, ++k)
       mat(j, i) = M_inv_[j](y, x).val[i];

       cv::Matx31d vec1(3, 1);
       for (int i = 0; i < 3; ++i)
       vec1(i, 0) = B.at<cv::Vec<T, 3> >(y, x).val[i];

       cv::Matx31d vec2 = mat * vec1;
       for (int i = 0; i < 3; ++i)
       {
       normals.at<cv::Vec<T, 3> >(y, x).val[i] = vec2(i, 0);
       }
       }
#endif

      return normals;
    }
    //TODO
    //private:
    int rows_;
    int cols_;
    int window_size_;
    cv::Matx<T, 3, 3> K_;

    cv::Mat_<cv::Vec<T, 3> > V_;
    // Each of the three elements is a row in M_inv
    std::vector<cv::Mat_<cv::Vec<T, 3> > > M_inv_;
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace
{
  template<typename T>
  class SRI: public cv::RgbdNormals::RgbdNormalsImpl
  {
  public:
    SRI(int rows, int cols, int window_size, const cv::Mat &K)
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
      cv::Mat cos_theta, sin_theta, cos_phi, sin_phi;
      computeThetaPhi<T>(rows_, cols_, K_, cos_theta, sin_theta, cos_phi, sin_phi);

      R_hat_.resize(3);
      for (unsigned char i = 0; i < 3; ++i)
        R_hat_[i].resize(3);

      cv::multiply(sin_theta, cos_phi, R_hat_[0][0]);
      R_hat_[0][1] = cos_theta;
      cv::multiply(-sin_theta, sin_phi, R_hat_[0][2]);

      R_hat_[1][0] = sin_phi;
      R_hat_[1][1] = cv::Mat_<T>::zeros(rows_, cols_);
      R_hat_[1][2] = cos_phi;

      cv::multiply(cos_theta, cos_phi, R_hat_[2][0]);
      R_hat_[2][1] = -sin_theta;
      cv::multiply(-cos_theta, sin_phi, R_hat_[2][2]);

#if 0
// Test to make sure we have a rotation matrix
      for (int y = 0; y < rows; ++y)
      for (int x = 0; x < cols; ++x)
      {
        cv::Mat_<float> R =
        (cv::Mat_<float>(3, 3) << R_hat_[0][0].at<float>(y, x), R_hat_[0][1].at<float>(y, x), R_hat_[0][2].at<float>(
                y, x), R_hat_[1][0].at<float>(y, x), R_hat_[1][1].at<float>(y, x), R_hat_[1][2].at<float>(y, x), R_hat_[2][0].at<
            float>(y, x), R_hat_[2][1].at<float>(y, x), R_hat_[2][2].at<float>(y, x));
        if (cv::norm(R * R.t(), cv::Mat::eye(3, 3, CV_32F)) > 1e-4)
        std::cout << R << std::endl;
      }
#endif

      for (unsigned char i = 0; i < 3; ++i)
        R_hat_[i][1] = R_hat_[i][1] / cos_phi;
    }

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
      cv::Mat r_theta, r_phi;
      cv::Sobel(r, r_theta, r.depth(), 1, 0, window_size_);
      cv::Sobel(r, r_phi, r.depth(), 0, 1, window_size_);
      tm1.stop();

      // Fill the result matrix
      tm2.start();
      std::vector<cv::Mat> res_channels(3);
      cv::divide(r_theta, r, r_theta);
      cv::divide(r_phi, r, r_phi);
      res_channels[0] = R_hat_[0][0] + R_hat_[0][1].mul(r_theta) + R_hat_[0][2].mul(r_phi);
      // R[1][1] is zero
      res_channels[1] = R_hat_[1][0] + R_hat_[1][2].mul(r_phi);
      res_channels[2] = R_hat_[2][0] + R_hat_[2][1].mul(r_theta) + R_hat_[2][2].mul(r_phi);
      tm2.stop();

      // Create the result matrix
      cv::Mat normals;
      cv::merge(res_channels, normals);

      std::cout << tm1.getTimeMilli() << " " << tm2.getTimeMilli() << " ";
      return normals;
    }
  private:
    int rows_;
    int cols_;
    int window_size_;
    cv::Matx<T, 3, 3> K_;

    /** Stores R */
    std::vector<std::vector<cv::Mat> > R_hat_;
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
    if (depth != CV_64F)
      depth = CV_32F;
    cv::Mat K_right_depth;
    K.convertTo(K_right_depth, depth);
    K_ = K_right_depth;

    // TODO make it a parameter
    int window_size = 5;
    if (method_ == RGBD_NORMALS_METHOD_SRI)
    {
      if (depth == CV_32F)
        rgbd_normals_impl_ = new SRI<float>(rows, cols, window_size, K_right_depth);
      else
        rgbd_normals_impl_ = new SRI<double>(rows, cols, window_size, K_right_depth);
    }
    else if (method_ == RGBD_NORMALS_METHOD_FALS)
    {
      if (depth == CV_32F)
      {
        std::cout << "**************";
        rgbd_normals_impl_ = new FALS<float>(rows, cols, window_size, K_right_depth);
      }
      else
        rgbd_normals_impl_ = new FALS<double>(rows, cols, window_size, K_right_depth);
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

    cv::Mat points3d;
    in_points3d.convertTo(points3d, K_.depth());

    tm_all.start();
    cv::Mat normals;

    tm1.start();
    cv::Mat r;
    r = computeR(points3d);
    tm1.stop();
    std::cout << "Time = " << tm1.getTimeMilli() << " ";

    if (method_ == RGBD_NORMALS_METHOD_SRI)
      normals = rgbd_normals_impl_->compute(r);
    else if (method_ == RGBD_NORMALS_METHOD_FALS)
      normals = rgbd_normals_impl_->compute(r);

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
