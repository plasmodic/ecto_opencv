/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#include <opencv2/rgbd/rgbd.hpp>
#include <iostream>
#include <limits>

#include "depth_to_3d.h"
#include "utils.h"

namespace
{
  /**
   * @param K
   * @param depth the depth image
   * @param mask the mask of the points to consider (can be empty)
   * @param points3d the resulting 3d points
   */
  void
  depthTo3d_from_uvz(const cv::Mat& in_K, const cv::Mat& u_mat, const cv::Mat& v_mat, const cv::Mat& z_mat,
                     cv::Mat& points3d)
  {
    CV_Assert((u_mat.size() == z_mat.size()) && (v_mat.size() == z_mat.size()));
    if (u_mat.empty())
    {
      points3d = cv::Mat();
      return;
    }
    CV_Assert((u_mat.type() == z_mat.type()) && (v_mat.type() == z_mat.type()));

    //grab camera params
    cv::Mat_<float> K;

    if (in_K.depth() == CV_32F)
      K = in_K;
    else
      in_K.convertTo(K, CV_32F);

    float fx = K(0, 0);
    float fy = K(1, 1);
    float s = K(0, 1);
    float cx = K(0, 2);
    float cy = K(1, 2);

    std::vector<cv::Mat> coordinates(3);

    coordinates[0] = (u_mat - cx) / fx;

    if (s != 0)
      coordinates[0] = coordinates[0] + (-(s / fy) * v_mat + cy * s / fy) / fx;

    coordinates[0] = coordinates[0].mul(z_mat);
    coordinates[1] = (v_mat - cy).mul(z_mat) * (1. / fy);
    coordinates[2] = z_mat;
    cv::merge(coordinates, points3d);
  }

  /**
   * @param K
   * @param depth the depth image
   * @param mask the mask of the points to consider (can be empty)
   * @param points3d the resulting 3d points
   */
  void
  depthTo3dMask(const cv::Mat& depth, const cv::Mat& K, const cv::Mat& mask, cv::Mat& points3d)
  {
    // Create 3D points in one go.
    cv::Size depth_size = depth.size();
    cv::Mat_<float> u_mat = cv::Mat_<float>(depth_size.area(), 1), v_mat = cv::Mat_<float>(depth_size.area(), 1),
        z_mat = cv::Mat_<float>(depth_size.area(), 1);

    cv::Mat_<uchar> uchar_mask = mask;

    if (mask.depth() != (CV_8U))
      mask.convertTo(uchar_mask, CV_8U);

    // Figure out the interesting indices
    size_t n_points;

    if (depth.depth() == CV_16U)
      n_points = convertDepthToFloat<uint16_t>(depth, mask, 1.0 / 1000.0f, u_mat, v_mat, z_mat);
    else if (depth.depth() == CV_16S)
      n_points = convertDepthToFloat<int16_t>(depth, mask, 1.0 / 1000.0f, u_mat, v_mat, z_mat);
    else
    {
      CV_Assert(depth.type() == CV_32F);
      n_points = convertDepthToFloat<float>(depth, mask, 1.0f, u_mat, v_mat, z_mat);
    }

    if (n_points == 0)
      return;

    u_mat.resize(n_points);
    v_mat.resize(n_points);
    z_mat.resize(n_points);

    depthTo3d_from_uvz(K, u_mat, v_mat, z_mat, points3d);
    points3d = points3d.reshape(3, 1);
  }

  /**
   * @param K
   * @param depth the depth image
   * @param points3d the resulting 3d points
   */
  template<typename T>
  void
  depthTo3dNoMask(const cv::Mat& in_depth, const cv::Mat_<T>& K, cv::Mat& points3d)
  {
    // Create 3D points in one go.
    cv::Size depth_size = in_depth.size();

    //grab camera params
    float fx = K(0, 0);
    float fy = K(1, 1);
    float s = K(0, 1);
    float cx = K(0, 2);
    float cy = K(1, 2);

    // Build z
    cv::Mat_<T> u_mat = cv::Mat_<T>(depth_size), v_mat = cv::Mat_<T>(depth_size), z_mat = cv::Mat_<T>(depth_size);
    rescaleDepthTemplated<T>(in_depth, z_mat);

    // Build the set of v's
    cv::Mat_<T> us = cv::Mat_<T>(1, depth_size.width), vs = cv::Mat_<T>(depth_size.height, 1);
    T* u_data = reinterpret_cast<T*>(us.data), *v_data = reinterpret_cast<T*>(vs.data);

    for (int u = 0; u < depth_size.width; ++u, ++u_data)
      *u_data = u;

    for (int v = 0; v < depth_size.height; ++v, ++v_data)
      *v_data = v;

    cv::repeat((vs - cy) / fy, 1, depth_size.width, v_mat);

    // Build the set of u's
    cv::repeat((us - cx) / fx, depth_size.height, 1, u_mat);

    if (s != 0)
    {
      cv::repeat(-(s / fx / fy) * vs, 1, depth_size.width, vs);
      u_mat = u_mat + vs + (cy * s / fy);
    }

    // Compute all the coordinates
    cv::Mat_<T> coordinates[3] =
    { u_mat.mul(z_mat), v_mat.mul(z_mat), z_mat };
    cv::Mat tmp_points;
    cv::merge(coordinates, 3, tmp_points);
    points3d = tmp_points.reshape(3, in_depth.rows);
  }
}

///////////////////////////////////////////////////////////////////////////////

namespace cv
{

  /**
   * @param K
   * @param depth the depth image
   * @param u_mat the list of x coordinates
   * @param v_mat the list of matching y coordinates
   * @param points3d the resulting 3d points
   */
  void
  depthTo3dSparse(const cv::Mat& depth, const cv::Mat& in_K, const cv::InputArray in_points, cv::Mat& points3d)
  {
    // Make sure we use foat types
    cv::Mat points = in_points.getMat();

    cv::Mat points_float;
    if (points.depth() != CV_32F)
      points.convertTo(points_float, CV_32FC2);
    else
      points_float = points;

    // Fill the depth matrix
    cv::Mat_<float> z_mat;

    if (depth.depth() == CV_16U)
      convertDepthToFloat<uint16_t>(depth, 1.0 / 1000.0f, points_float, z_mat);
    else if (depth.depth() == CV_16U)
      convertDepthToFloat<int16_t>(depth, 1.0 / 1000.0f, points_float, z_mat);
    else
    {
      CV_Assert(depth.type() == CV_32F);
      convertDepthToFloat<float>(depth, 1.0f, points_float, z_mat);
    }

    std::vector<cv::Mat> channels(2);
    cv::split(points_float, channels);

    depthTo3d_from_uvz(in_K, channels[0], channels[1], z_mat, points3d);
  }

  /**
   * @param depth the depth image (if given as short int CV_U, it is assumed to be the depth in millimeters
   *              (as done with the Microsoft Kinect), otherwise, if given as CV_32F, it is assumed in meters)
   * @param K The calibration matrix
   * @param points3d the resulting 3d points as a cv::Mat of the same size but containing cv::Vec3f
   * @param mask the mask of the points to consider (can be empty)
   */
  void
  depthTo3d(const cv::Mat& depth, const cv::Mat& K, cv::Mat& points3d, const cv::Mat& mask)
  {
    CV_Assert(K.cols == 3 && K.rows == 3 && (K.depth() == CV_64F || K.depth()==CV_32F));
    CV_Assert(
        depth.type() == CV_64FC1 || depth.type() == CV_32FC1 || depth.type() == CV_16UC1 || depth.type() == CV_16SC1);
    CV_Assert(K.cols == 3 && K.rows == 3);
    CV_Assert(mask.channels() == 1);

    // TODO figure out what to do when types are different: convert or reject ?
    if ((depth.depth() == CV_32F || depth.depth() == CV_64F) && depth.depth() != K.depth())
    {
      if (K.depth() == CV_32F)
        K.convertTo(K, CV_32F);
    }

    // Create 3D points in one go.
    if (!mask.empty())
      depthTo3dMask(depth, K, mask, points3d);
    else
    {
      if (K.depth() == CV_64F)
        depthTo3dNoMask<double>(depth, K, points3d);
      else
        depthTo3dNoMask<float>(depth, K, points3d);
    }

  }
}
