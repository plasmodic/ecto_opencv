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
      coordinates[0] = coordinates[0] + ( - (s / fy) * v_mat + cy * s / fy) / fx;

    coordinates[0] = coordinates[0].mul(z_mat);
    coordinates[1] = (v_mat - cy).mul(z_mat)  * (1. / fy);
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
    unsigned int n_points = 0;
    const uchar* r;

    if (depth.depth() == CV_16U)
    {
      // Raw data from the Kinect has int
      for (int v = 0; v < depth_size.height; v++)
      {
        r = uchar_mask.ptr<uchar>(v, 0);

        for (int u = 0; u < depth_size.width; u++, ++r)
          if (*r)
          {
            u_mat(n_points, 0) = u;
            v_mat(n_points, 0) = v;
            uint16_t depth_i = depth.at<uint16_t>(v, u);

            if ((depth_i == std::numeric_limits<uint16_t>::min()) || (depth_i == std::numeric_limits<uint16_t>::max()))
              z_mat(n_points, 0) = std::numeric_limits<float>::quiet_NaN();
            else
              z_mat(n_points, 0) = depth_i / 1000.0f;

            ++n_points;
          }
      }
    }
    else
    {
      // If the depth is already floats, no need to divide by 1000
      for (int v = 0; v < depth_size.height; v++)
      {
        r = uchar_mask.ptr<uchar>(v, 0);

        for (int u = 0; u < depth_size.width; u++, ++r)
          if (*r)
          {
            u_mat(n_points, 0) = u;
            v_mat(n_points, 0) = v;
            float z = depth.at<float>(v, u);

            if ((z != z) || (z == std::numeric_limits<float>::max()))
              z_mat(n_points, 0) = std::numeric_limits<float>::quiet_NaN();
            else
              z_mat(n_points, 0) = z;

            ++n_points;
          }
      }
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
  void
  depthTo3dNoMask(const cv::Mat& in_depth, const cv::Mat& in_K, cv::Mat& points3d)
  {
    // Create 3D points in one go.
    cv::Size depth_size = in_depth.size();
    cv::Mat_<float> u_mat = cv::Mat_<float>(depth_size), v_mat = cv::Mat_<float>(depth_size), z_mat = cv::Mat_<float>(
                              depth_size);

    // If we compute on the whole matrix
    cv::Mat_<float> K;
    in_K.convertTo(K, CV_32F);

    //grab camera params
    float fx = K.at<float>(0, 0);
    float fy = K.at<float>(1, 1);
    float s = K.at<float>(0, 1);
    float cx = K.at<float>(0, 2);
    float cy = K.at<float>(1, 2);

    // Build z
    rescaleDepth(in_depth, z_mat);

    // Build the set of v's
    cv::Mat_<float> us = cv::Mat_<float>(1, depth_size.width), vs = cv::Mat_<float>(depth_size.height, 1);
    float* u_data = us.ptr<float>(0, 0), *v_data = vs.ptr<float>(0, 0);

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
    cv::Mat_<float> coordinates[3] =
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
  depthTo3dSparse(const cv::Mat& in_K, const cv::Mat& depth, const cv::Mat& u_mat, const cv::Mat& v_mat,
                  cv::Mat& points3d)
  {
    // Make sure we use foat types
    cv::Mat_<float> u_float, v_float;

    if (u_mat.depth() == (CV_32F))
      u_float = u_mat;
    else
      u_mat.convertTo(u_float, CV_32F);

    if (v_mat.depth() == (CV_32F))
      v_float = v_mat;
    else
      v_mat.convertTo(v_float, CV_32F);

    // Fill the depth matrix
    cv::Mat_<float> z_float = cv::Mat_<float>(u_float.rows, u_float.cols);

    cv::Mat_<float>::const_iterator iter_v = v_float.begin(), iter_u = u_float.begin(), iter_u_end = u_float.end();
    float* iter_z = reinterpret_cast<float*>(z_float.data);

    if (depth.depth() == CV_16U)
    {
      for (; iter_u != iter_u_end; ++iter_u, ++iter_v, ++iter_z)
      {
        uint16_t depth_i = depth.at<uint16_t>(*iter_v, *iter_u);

        if ((depth_i == 0) || (depth_i == std::numeric_limits<uint16_t>::max()))
          *iter_z = std::numeric_limits<float>::quiet_NaN();
        else
          *iter_z = depth_i / 1000.0f;
      }
    }
    else
      if (depth.depth() == CV_32F)
      {
        for (; iter_u != iter_u_end; ++iter_u, ++iter_v, ++iter_z)
          *iter_z = depth.at<float>(*iter_v, *iter_u);
      }

    depthTo3d_from_uvz(in_K, u_float, v_float, z_float, points3d);
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
    CV_Assert(K.cols == 3 && K.rows == 3);
    CV_Assert(depth.type() == CV_32FC1 || depth.type() == CV_16SC1);
    CV_Assert(mask.channels() == 1);

    // Create 3D points in one go.
    if (!mask.empty())
      depthTo3dMask(depth, K, mask, points3d);
    else
      depthTo3dNoMask(depth, K, points3d);
  }
}
