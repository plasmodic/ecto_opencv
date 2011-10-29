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

namespace projector
{
  void
  solvePlaneSVD(cv::Mat xyz, cv::Mat& plane)
  {
//    points is 3,N
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
      typedef PlaneFitter C;
      inputs.declare(&C::points3d_in, "points3d", "The 3d points to estimate the plane from.");
      outputs.declare(&C::R_out, "R", "The output R vec");
      outputs.declare(&C::T_out, "T", "The output T vec");
      outputs.declare(&C::plane_out, "plane", "Plane model coefficients.");
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
    process(const tendrils& inputs, const tendrils& outputs)
    {
      cv::Mat plane;
      cv::Mat points3d = *points3d_in;
      if (points3d.empty()) return ecto::OK;
      points3d =  points3d.reshape(1,points3d.rows).t();
      solvePlaneSVD(points3d, plane);
      cv::Mat_<float> R, T;
      solveRT(plane, R, T);
      *R_out = R;
      *T_out = T;
      *plane_out = plane;
      return ecto::OK;
    }
    ecto::spore<cv::Mat> points3d_in, R_out, T_out;
    ecto::spore<std::vector<float> > plane_out;
  };
}
ECTO_CELL(calib, projector::PlaneFitter, "PlaneFitter",
          "Fits a plane in least squares manner to a set of 3d points.");
