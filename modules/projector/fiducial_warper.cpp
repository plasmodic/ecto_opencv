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

#include <math.h>

#include <boost/foreach.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>

#include "common.h"

using ecto::tendrils;

cv::Mat_<float>
calcH(const cv::Mat_<float>& R, const cv::Mat_<float>& T, const cv::Mat_<float>& P, float scale, float cx, float cy)
{
  cv::Mat_<float> H, A = (cv::Mat_<float>(4, 3) << R(0, 0), R(0, 1), T(0), //
  R(1, 0), R(1, 1), T(1), //
  R(2, 0), R(2, 1), T(2), //
  0, 0, 1);
  cv::Mat_<float> offset = (cv::Mat_<float>(3, 3) << 1, 0, cx, 0, -1, cy, 0, 0, 1);
  H = P * A;
  cv::Mat_<float> SR = H.colRange(0, 2);
  SR *= scale;
  H = H * offset;
  return H;
}

cv::Mat_<float>
calcH(const cv::Mat_<float>& R, const cv::Mat_<float>& T, const cv::Mat_<float>& P, const cv::Mat_<float>& K)
{
  cv::Mat_<float> H, A = (cv::Mat_<float>(4, 3) << R(0, 0), R(0, 1), T(0), //
  R(1, 0), R(1, 1), T(1), //
  R(2, 0), R(2, 1), T(2), //
  0, 0, 1);
  H = P * A * K;
  return H;
}

//cv::Mat_<float>
//calcH_projector_to_camera(const cv::Mat_<float>& P, const cv::Mat_<float>& K)
//{
//  cv::Mat_<float> Pinv = P.inv(cv::DECOMP_SVD);
//  std::cout << "Pinv = " << Pinv << std::endl;
//  std::cout << "K = " << K << std::endl;
//  cv::Mat_<float> IO = (cv::Mat_<float>(3, 4) << 1, 0, 0, 0, //
//  0, 1, 0, 0, //
//  0, 0, 1, 0 //
//      );
//  return (K * IO) * Pinv;
//}

template<typename Cell>
struct Warper
{
  typedef std::vector<cv::Point2f> points_t;
  static void
  declare_params(tendrils& p)
  {
    p.declare<std::string>("projection_file");
    p.declare<int>("width").set_default_val(640);
    p.declare<int>("height").set_default_val(480);
    p.declare<float>("offset_x", "The X offset", 0);
    p.declare<float>("offset_y", "The Y offset", 0);
    Cell::declare_params(p);
  }

  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat>("R", "The original 2D pattern").required(true);
    inputs.declare<cv::Mat>("T", "The points we want to 3d-fy (an aternative to the keypoints)").required(true);
    inputs.declare<bool>("found", "The calibration matrix", true);
    outputs.declare<cv::Mat>("output", "The depth image");
    Cell::declare_io(params, inputs, outputs);
  }

  void
  configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    std::string projection_file;
    params["projection_file"] >> projection_file;
    readOpenCVCalibration(P_, projection_file);
    params["offset_x"] >> offset_x_;
    params["offset_y"] >> offset_y_;
    params["width"] >> width;
    params["height"] >> height;
    cell.configure(params, inputs, outputs);
  }

  /** Get the 2d keypoints and figure out their 3D position from the depth map
   * @param inputs
   * @param outputs
   * @return
   */
  int
  process(const tendrils& inputs, const tendrils& outputs)
  {
    bool found;
    inputs["found"] >> found;
    if (!found)
      return 0;
    cv::Mat R, T;
    inputs.get<cv::Mat>("R").convertTo(R, CV_32F);
    inputs.get<cv::Mat>("T").convertTo(T, CV_32F);
    // Draw the final image
    cv::Mat drawn_image = cv::Mat::zeros(height, width, CV_8UC3);
    cell.process(inputs, outputs, drawn_image, R, T, P_, offset_x_, offset_y_);
    outputs.get<cv::Mat>("output") = drawn_image;
    return 0;
  }
private:
  cv::Mat P_;
  float offset_x_;
  float offset_y_;
  int width, height;
  Cell cell;
};
/** Ecto implementation of a module that takes
 *
 */
struct FiducialWarper
{
  typedef std::vector<cv::Point2f> points_t;
  static void
  declare_params(tendrils& p)
  {
    p.declare<float>("radius", "The radius of the circle", 0.15);
  }

  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
  }

  void
  configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    radius = params.get<float>("radius");
  }

  /** Get the 2d keypoints and figure out their 3D position from the depth map
   * @param inputs
   * @param outputs
   * @return
   */
  int
  process(const tendrils& inputs, const tendrils& outputs, cv::Mat& draw_image, const cv::Mat_<float>& R,
          const cv::Mat_<float>& T, const cv::Mat_<float>& P, int offset_x, int offset_y)
  {
    std::vector<cv::Point3f> points_3d_vec;
    // Buld a circle
    for (float i = 0; i < 2 * CV_PI; i += 0.1)
      points_3d_vec.push_back(cv::Point3f(offset_x + radius * cos(i), offset_y + radius * sin(i), 0));

    // And ad a square around it
    points_3d_vec.push_back(cv::Point3f(offset_x + radius, offset_y + radius, 0));
    points_3d_vec.push_back(cv::Point3f(offset_x - radius, offset_y + radius, 0));
    points_3d_vec.push_back(cv::Point3f(offset_x - radius, offset_y - radius, 0));
    points_3d_vec.push_back(cv::Point3f(offset_x + radius, offset_y - radius, 0));
    points_3d_vec.push_back(points_3d_vec[0]);
    int n_points = points_3d_vec.size();

    // Compute the 3D warped points
    cv::Mat_<float> points_3d = cv::Mat(points_3d_vec).reshape(1, n_points).t();
    cv::Mat_<float> points_kinect = R * points_3d + T * cv::Mat_<float>::ones(1, n_points);
    points_kinect.resize(4, cv::Scalar(1));
    // Project them to 2d
    cv::Mat points_homogeneous = P * points_kinect;
    cv::Mat_<float> points_2d;
    cv::convertPointsFromHomogeneous(points_homogeneous.t(), points_2d);

    // Draw the final image
    for (int i = 0; i < points_2d.rows - 1; ++i)
      cv::line(draw_image, cv::Point2f(points_2d(i, 0), points_2d(i, 1)),
               cv::Point2f(points_2d(i + 1, 0), points_2d(i + 1, 1)), cv::Scalar(0, 255, 0), 10);
    return 0;
  }
  float radius;
};

struct ImageWarper
{
  typedef std::vector<cv::Point2f> points_t;
  static void
  declare_params(tendrils& p)
  {
    p.declare<float>("scale", "scale in meters.", 0.5);
  }

  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat>("image", "An input image to draw rectified.");
  }

  void
  configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    params["scale"] >> scale_;
  }

  /** Get the 2d keypoints and figure out their 3D position from the depth map
   * @param inputs
   * @param outputs
   * @return
   */
  int
  process(const tendrils& inputs, const tendrils& outputs, cv::Mat& draw_image, const cv::Mat_<float>& R,
          const cv::Mat_<float>& T, const cv::Mat_<float>& P, int offset_x, int offset_y)
  {
    cv::Mat image;
    inputs["image"] >> image;
    float scale = std::max(scale_ / image.size().width, scale_ / image.size().height);
    cv::Mat_<float> H = calcH(R, T, P, scale, -image.size().width / 2, image.size().height / 2);
    cv::warpPerspective(image, draw_image, H, draw_image.size());
    return 0;
  }
  float scale_;
};

struct CameraWarper
{
  typedef std::vector<cv::Point2f> points_t;
  static void
  declare_params(tendrils& p)
  {
  }

  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat>("image", "An input image to draw rectified.").required(true);
    inputs.declare<cv::Mat>("K", "Camera matrix.").required(true);
  }

  void
  configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
  }
  /** Get the 2d keypoints and figure out their 3D position from the depth map
   * @param inputs
   * @param outputs
   * @return
   */
  int
  process(const tendrils& inputs, const tendrils& outputs, cv::Mat& draw_image, const cv::Mat_<float>& R,
          const cv::Mat_<float>& T, const cv::Mat_<float>& P, int offset_x, int offset_y)
  {
    cv::Mat image;
    inputs["image"] >> image;
    cv::Mat K;
    inputs["K"] >> K;
    cv::Mat_<float> H = calcH(P, K, R, T);
    cv::warpPerspective(image, draw_image, H, draw_image.size());
    return 0;
  }
  float scale_;
};

ECTO_CELL(projector, Warper<FiducialWarper>, "FiducialWarper", "Figures out the calibration of the projector.");
ECTO_CELL(projector, Warper<ImageWarper>, "ImageWarper", "Figures out the calibration of the projector.");
ECTO_CELL(projector, Warper<CameraWarper>, "CameraWarper", "Warps from projector space to camera space");
