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

#include <ecto/ecto.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>

#include "common.h"

namespace projector
{
  void
  depth23d(const cv::Mat& K, const cv::Mat& depth, cv::Mat& points3d, const cv::Rect& roi)
  {
    cv::Mat_<float> scaled_points = cv::Mat_<float>(depth.size().area(), 3);
    // Create the scaled keypoints
    cv::Size depth_size = depth.size();
    cv::Mat_<float>::const_iterator begin = depth.begin<float>(), end = depth.end<float>();
    cv::Mat_<float>::iterator sp_begin = scaled_points.begin();
    cv::Point2f point = roi.tl();
    while (begin != end)
    {
      float d = *(begin++);
      *(sp_begin++) = point.x * d;
      *(sp_begin++) = point.y * d;
      *(sp_begin++) = d;
      if (point.x < roi.br().x)
      {
        point.x += 1;
      }
      else
      {
        point.y += 1;
        point.x = roi.tl().x;
      }
    }
    // Figure out the 3D points
    cv::Mat_<float> final_points_tmp;
    cv::solve(K, scaled_points.t(), final_points_tmp);
    points3d = final_points_tmp;
  }
  /**
   *
   * @param K
   * @param depth a subregion of a matrix.
   * @param points3d
   * @param roi the roi that create the subregion.
   */
  void
  depthTo3d(const cv::Mat& K, const cv::Mat& depth, cv::Mat& points3d, const cv::Rect& roi)
  {
    //allocate points.
    cv::Mat_<float> points = cv::Mat_<float>(depth.size().area(), 3);
    cv::Mat_<float>::iterator sp_begin = points.begin();
    //grab camera params
    float fx = K.at<float>(0, 0);
    float fy = K.at<float>(1, 1);
    float cx = K.at<float>(0, 2);
    float cy = K.at<float>(1, 2);
    // std::cout << fx << " "<< fy << std::endl;
    // Create 3D points in one go.
    cv::Size depth_size = depth.size();
    for (int v = 0; v < depth_size.height; v++)
    {
      for (int u = 0; u < depth_size.width; u++)
      {
        cv::Point p(u, v);
        cv::Point rp = p + roi.tl();
        float z = depth.at<float>(p.y, p.x);
        *(sp_begin++) = (rp.x - cx) * z / fx;
        *(sp_begin++) = (rp.y - cy) * z / fy;
        *(sp_begin++) = z;
      }
    }
    points3d = points.t(); //return 3xN matrix
  }
  void
  solvePlane(cv::Mat xyz, cv::Mat& plane)
  {
    //points is 3,N
//    % Set up constraint equations of the form  AB = 0,
//    % where B is a column vector of the plane coefficients
//    % in the form   b(1)*X + b(2)*Y +b(3)*Z + b(4) = 0.
//
//    A = [XYZ' ones(npts,1)]; % Build constraint matrix
//
//    if npts == 3             % Pad A with zeros
//      A = [A; zeros(1,4)];
//    end
//
//    [u d v] = svd(A);        % Singular value decomposition.
//    B = v(:,4);              % Solution is last column of v.
    cv::Mat A = xyz;
    A.resize(4, cv::Scalar(1));
    cv::SVD svd(A.t());
    plane = svd.vt.row(svd.vt.rows - 1);
  }

  template<typename T>
  int
  sign(T f)
  {
    if (f > 0)
      return 1;
    else
      return -1;
  }

  void
  solveRT(cv::Mat_<float> plane, cv::Mat_<float>& R, cv::Mat_<float>& T)
  {
    float /*a = plane(0), b = plane(1),*/c = plane(2), d = plane(3);
    float z = -d / c;
    cv::Mat_<float> normal = plane.colRange(0, 3).t();
    normal = normal / cv::norm(normal);
    T = (cv::Mat_<float>(3, 1) << 0, 0, z);
    //construct an ortho normal basis.
    cv::Mat_<float> X = (cv::Mat_<float>(3, 1) << 1, 0, 0); //unit x vector.
    cv::Mat_<float> Vx = X - X.dot(normal) * normal;
    cv::Mat_<float> Y = (cv::Mat_<float>(3, 1) << 0, 1, 0);
    cv::Mat_<float> Vy = (cv::Mat_<float>(3, 1) << 0, -1, 0); //make y go up.
    Vy = Vy - Vy.dot(normal) * normal;
    Vx = Vx / cv::norm(Vx);
    Vy = Vy / cv::norm(Vy);

    cv::Mat_<float> Z = (cv::Mat_<float>(3, 1) << 0, 0, 1);
    cv::Mat_<float> Vz = Vx.cross(Vy); //construct a right handed basis. Z = X cross Y

    //Build up correlation matrix
    cv::Mat_<float> C = cv::Mat_<float>::zeros(3, 3);
    C += Vx * X.t();
    C += Vy * Y.t();
    C += Vz * Z.t();

    //solve for R using svd
    cv::SVD svd(C);

    //sgn of the determinant
    int s = sign(cv::determinant(svd.u * svd.vt));

    cv::Mat_<float> diagm = cv::Mat_<float>::eye(3, 3);
    //create a matrix with all ones but the lower right corner = S
    diagm(2, 2) = s;

    //according to the paper TM = U * diag(1,1,s) * V^T
    R = svd.u * diagm * svd.vt;
  }
  using ecto::tendrils;

  struct PlaneFitter
  {
    typedef std::vector<cv::Point2f> points_t;
    static void
    declare_params(tendrils& p)
    {
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Mat>("K", "The calibration matrix");
      inputs.declare<cv::Mat>("depth", "The depth image");
      outputs.declare<cv::Mat>("R", "The output R vec");
      outputs.declare<cv::Mat>("T", "The output T vec");
      outputs.declare<bool>("found", "Found a plane", true);

    }

    void
    configure(tendrils& params, tendrils& inputs, tendrils& outputs)
    {
    }

    /** Get the 2d keypoints and figure out their 3D position from the depth map
     * @param inputs
     * @param outputs
     * @return
     */
    int
    process(tendrils& inputs, tendrils& outputs)
    {
      cv::Mat K, depth;
      inputs["K"] >> K;
      inputs["depth"] >> depth;
      K.clone().convertTo(K, CV_32F);
      cv::Mat points3d;
      int roi_size = 40; //sets the sample region MxM
      cv::Rect roi(depth.size().width / 2 - roi_size / 2, depth.size().height / 2 - roi_size / 2, roi_size, roi_size);
      cv::Mat depth_sub = depth(roi); //grab the sample region
      depthTo3d(K, depth_sub, points3d, roi);
      cv::Mat plane;
      //std::cout <<"points: " << points3d.t() << std::endl;
      solvePlane(points3d, plane);
      //std::cout << "Plane = " << plane << std::endl;
      cv::Mat_<float> R, T;
      solveRT(plane, R, T);
      //std::cout << "R = " << R << std::endl;
      //std::cout << "T = " << T << std::endl;
      outputs["R"] << cv::Mat(R);
      outputs["T"] << cv::Mat(T);
      return 0;
    }
  };
}
ECTO_CELL(projector, projector::PlaneFitter, "PlaneFitter", "Finds the plane.");
