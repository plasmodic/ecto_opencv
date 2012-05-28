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
    cv::Mat r;
    points.copyTo(r);
    r = r.reshape(1, r.cols * r.rows);
    cv::multiply(r, r, r);
    cv::reduce(r, r, 1, CV_REDUCE_SUM);
    cv::sqrt(r, r);
    r = r.reshape(1, points.rows);

    return r;
  }

  // Compute theta and phi according to equation 3
  template<typename T>
  void
  computeThetaPhi(int rows, int cols, const cv::Mat& points, cv::Mat &cos_theta, cv::Mat &sin_theta, cv::Mat &phi,
                  cv::Mat &cos_phi, cv::Mat &sin_phi)
  {
    typedef cv::Matx<T, 3, 1> Vec_T;

    cos_theta = cv::Mat_<T>(rows, cols);
    sin_theta = cv::Mat_<T>(rows, cols);
    cos_phi = cv::Mat_<T>(rows, cols);
    sin_phi = cv::Mat_<T>(rows, cols);
    phi = cv::Mat_<T>(rows, cols);
    cv::Mat r = computeR(points);
    for (int y = 0; y < rows; ++y)
    {
      T * row_cos_theta = cos_theta.ptr<T>(y), *row_sin_theta = sin_theta.ptr<T>(y);
      T *row_cos_phi = cos_phi.ptr<T>(y), *row_sin_phi = sin_phi.ptr<T>(y);
      T *row_phi = phi.ptr<T>(y);
      const Vec_T * row_points = points.ptr<Vec_T>(y), *row_points_end = points.ptr<Vec_T>(y) + points.cols;
      const T * row_r = r.ptr<T>(y);
      for (; row_points < row_points_end;
          ++row_cos_theta, ++row_sin_theta, ++row_cos_phi, ++row_sin_phi, ++row_phi, ++row_points, ++row_r)
      {
        // In the paper, x goes away from the camera, y goes down, z goes left
        // In our convention, x goes right, y goes down, z goes away from the camera
        // We therefore need to replace the following paper notations:
        // x->z, y->y, z->-x
        float theta = std::atan2(row_points->val[2], -row_points->val[0]);
        *row_cos_theta = std::cos(theta);
        *row_sin_theta = std::sin(theta);
        *row_phi = std::asin(row_points->val[1] / (*row_r));
        *row_cos_phi = std::cos(*row_phi);
        *row_sin_phi = std::sin(*row_phi);
      }
    }
  }
}

namespace cv
{
  /** Default constructor
   */
  RgbdNormals::RgbdNormals(int rows, int cols, int depth, const cv::Mat & K)
  {
    // Create some bogus coordinates
    cv::Mat depth_image = cv::Mat::ones(rows, cols, depth);
    cv::Mat K_right_depth;
    K.convertTo(K_right_depth, depth);
    cv::Mat points3d;
    depthTo3d(depth_image, K_right_depth, points3d);

    // Compute theta and phi according to equation 3
    cv::Mat cos_theta, sin_theta, phi, cos_phi, sin_phi;
    std::vector<cv::Mat> xyz(3);
    cv::split(points3d, xyz);
    if (depth == CV_32F)
      computeThetaPhi<float>(rows, cols, points3d, cos_theta, sin_theta, phi, cos_phi, sin_phi);
    else
      computeThetaPhi<double>(rows, cols, points3d, cos_theta, sin_theta, phi, cos_phi, sin_phi);

    cos_phi_inv_ = cv::Mat::ones(rows, cols, depth) / phi;

    R_hat_.resize(3);
    for (unsigned char i = 0; i < 3; ++i)
      R_hat_[i].resize(3);

    cv::multiply(sin_theta, cos_phi, R_hat_[0][0]);
    R_hat_[0][1] = cos_theta;
    cv::multiply(-sin_theta, sin_phi, R_hat_[0][2]);

    R_hat_[1][0] = sin_phi;
    R_hat_[1][1] = cv::Mat::zeros(rows, cols, depth);
    R_hat_[1][2] = cos_phi;

    cv::multiply(cos_theta, cos_phi, R_hat_[2][0]);
    R_hat_[2][1] = -sin_theta;
    cv::multiply(-cos_theta, sin_phi, R_hat_[2][2]);
  }

  /** Given a set of 3d points in a depth image, compute the normals at each point
   * using the SRI method described in
   * ``Fast and Accurate Computation of Surface Normals from Range Images``
   * by H. Badino, D. Huber, Y. Park and T. Kanade
   * @param points a rows x cols x 3 matrix
   * @return normals a rows x cols x 3 matrix
   */
  cv::Mat
  RgbdNormals::operator()(const cv::Mat &points) const
  {
    CV_Assert(points.channels()==3 && points.dims==2);
    CV_Assert(points.depth()==CV_32F || points.depth()==CV_64F);

    int rows = points.rows, cols = points.cols, depth = points.depth();
    std::vector<cv::Mat> xyz(3);
    cv::split(points, xyz);

    // Compute r
    cv::Mat r = computeR(points);

    // Compute the derivatives with respect to theta and phi
    cv::Mat r_theta, r_phi;
    cv::Scharr(r, r_theta, depth, 1, 0);
    cv::Scharr(r, r_phi, depth, 0, 1);

    // Fill the result matrix
    cv::Mat r_inv;
    cv::divide(cv::Mat::ones(rows, cols, depth), r, r_inv);

    std::vector<cv::Mat> res_channels(3);
    for (unsigned char i = 0; i < 3; ++i)
    {
      cv::Mat tmp1, tmp2;
      cv::multiply(R_hat_[i][1], cos_phi_inv_, tmp1);
      cv::multiply(tmp1, r_inv, tmp1);
      cv::multiply(R_hat_[i][2], r_inv, tmp2);

      res_channels[i] = R_hat_[i][0] + tmp1 + tmp2;
    }

    // Create the result matrix
    cv::Mat res;
    cv::merge(res_channels, res);

    return res;
  }
}
