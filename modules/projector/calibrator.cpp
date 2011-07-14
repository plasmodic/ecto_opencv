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

#include <boost/foreach.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>

#include "common.h"

using ecto::tendrils;

struct Camera
{
  cv::Mat K, D;
  cv::Size image_size;
};

cv::Mat_<float> kronecker(const cv::Mat_<float> &A, const cv::Mat_<float> &B)
{
  cv::Mat_<float> C(A.rows * B.rows, A.cols * B.cols);
  for (int j = 0, j_block = 0; j < A.rows; ++j, j_block += B.rows)
    for (int i = 0, i_block = 0; i < A.cols; ++i, i_block += B.cols)
    {
      cv::Mat C_block = C(cv::Range(j_block, j_block + B.rows), cv::Range(i_block, i_block + B.cols));
      cv::Mat_<float>(A(j, i) * B).copyTo(C_block);
    }
  return C;
}

void solve_for_P(const cv::Mat & points_2d, const cv::Mat &points_3d, cv::Mat & P)
{
  int n = points_2d.cols;

  cv::Mat_<float> one_vect = cv::Mat_<float>::zeros(n + 1, 1);
  one_vect(0, 0) = 1;

  cv::Mat_<float> C = kronecker(cv::Mat_<float>::eye(n, n), one_vect);
  C.resize(n * n);

  cv::Mat_<float> D(3 * n, n + 12);
  cv::Mat_<float> D_left = D.colRange(0, n);
  cv::Mat_<float> tmp = kronecker(cv::Mat_<float>::eye(n, n), points_2d);

  cv::Mat_<float>(cv::Mat_<float>(kronecker(cv::Mat_<float>::eye(n, n), points_2d)) * C).copyTo(D_left);
  cv::Mat_<float> D_right = D.colRange(n, n + 12);
  cv::Mat_<float>(-kronecker(points_3d.t(), cv::Mat_<float>::eye(3, 3))).copyTo(D_right);

  cv::SVD svd;
  svd(D);
  cv::Mat_<float> P_row = cv::Mat(svd.vt.row(svd.vt.rows - 1).colRange(n, n + 12)).clone();

  P = (cv::Mat(P_row)).reshape(1, 4).t();
  std::cout << P << std::endl;
}

void solve_for_P_old(const cv::Mat & points_2d, const cv::Mat &points_3d, cv::Mat & P)
{
  cv::Mat Pt;
  cv::solve(points_3d, points_2d, Pt, cv::DECOMP_SVD);
  P = Pt.t();
}

/** Ecto implementation of a module that takes
 *
 */
struct Calibrator
{
  typedef std::vector<cv::Point2f> points_t;
  static void declare_params(tendrils& p)
  {
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<points_t>("pattern", "The original 2D pattern");
    inputs.declare<points_t>("points", "The points we want to 3d-fy (an aternative to the keypoints)");
    inputs.declare<cv::Mat>("K", "The calibration matrix");
    inputs.declare<cv::Mat>("depth", "The depth image");
    inputs.declare<bool>("found", "Whether the pattern aws found");
    inputs.declare<int>("trigger", "Trigger a capture, 'c' for capture.");
    outputs.declare<cv::Mat>("P", "The 3x4 projection matrix");
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
  }

  /** Get the 2d keypoints and figure out their 3D position from the depth map
   * @param inputs
   * @param outputs
   * @return
   */
  int process(tendrils& inputs, tendrils& outputs)
  {
    bool found;
    int trigger;
    inputs["found"] >> found;
    inputs["trigger"] >> trigger;

    if (!found || trigger != 's')
    {
      return 0;
    }
    inputs["trigger"] << 0; //clear the trigger.

    points_t pattern_vector, points;
    inputs["pattern"] >> pattern_vector;
    inputs["points"] >> points;

    cv::Mat K, depth;
    inputs["K"] >> K;
    inputs["depth"] >> depth;

    K.clone().convertTo(K, CV_32F);
    // We have lam (x,y,1) = K (X,Y,Z), hence lam=Z
    //const std::vector<cv::Point2f> &points = inputs.get<std::vector<cv::KeyPoint> >("keypoints");
    //const std::vector<cv::KeyPoint> &keypoints = inputs.get<std::vector<cv::KeyPoint> >("keypoints");
    //const cv::Mat & depth_image = inputs.get<cv::Mat>("depth");

    cv::Mat_<float> scaled_points;

    // Create the scaled keypoints
    int i = 0;
    scaled_points = cv::Mat_<float>(3, points.size());
    BOOST_FOREACH(const cv::Point2f & point, points)
        {
          float d = depth.at<float>(point.y, point.x);
          scaled_points(0, i) = point.x * d;
          scaled_points(1, i) = point.y * d;
          scaled_points(2, i) = d;
          ++i;
        }

    // Figure out the 3D points
    cv::Mat_<float> final_points_tmp;
    cv::solve(K, scaled_points, final_points_tmp);
    final_points_tmp.resize(4, cv::Scalar(1));

    static cv::Mat_<float> final_points;
    static std::vector<std::vector<cv::Point3f> > final_points_calib;
    static cv::Mat pattern;
    static std::vector<std::vector<cv::Point2f> > pattern_calib;

    cv::Mat_<float> pattern_tmp = cv::Mat(pattern_vector).reshape(1).t();
    pattern_tmp.resize(3, cv::Scalar(1));
    pattern_tmp = pattern_tmp.t();

    if (final_points.empty())
    {
      final_points = final_points_tmp.t();
      pattern = pattern_tmp;
    }
    else if (final_points.rows < 2000)
    {
      final_points.push_back((const cv::Mat &)(final_points_tmp.t()));
      pattern.push_back((const cv::Mat &)(pattern_tmp));
    }

    cv::Mat_<float> P1, P2, P3, distortion;
    solve_for_P(pattern.t(), final_points.t(), P1);
    solve_for_P_old(pattern, final_points, P2);
    P1 = P1 / P1(2, 3);
    P2 = P2 / P2(2, 3);
    //std::cout << P1 << std::endl;
    //std::cout << P2 << std::endl;

    writeOpenCVCalibration(P1, "projector_calibration.yml");
    //outputs.get<cv::Mat>("P") = P;

    return 0;
  }
};

ECTO_CELL(projector, Calibrator, "Calibrator", "Figures out the calibration of the projector.");
