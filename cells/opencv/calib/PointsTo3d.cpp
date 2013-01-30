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

#include <limits>

#include <ecto/ecto.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

using ecto::tendrils;
namespace calib
{
  struct PointsTo3d
  {
    typedef PointsTo3d C;

    static void
    declare_params(tendrils& params)
    {
      params.declare(&C::transpose, "transpose", "Transpose the input, to accept a 2xN matrix.", false);
      params.declare(&C::scale_factor, "scale", "Pixel to metric scale factor. u * scale = x, v * scale = y", 1.0);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&C::points, "points", "A Nx2 matrix, of real values.").required(true);
      outputs.declare(&C::points3d, "points3d",
                      "The 3d points, same dimensions as the input, 3 channels (x, y and z).");
    }

    template<typename T>
    static void
    convert(T scale, cv::Mat & _m)
    {
      cv::Mat_<float> m(_m);
      for (size_t i = 0, end = m.rows; i != end; i++)
      {
        m(i, 0) *= scale;
        m(i, 1) *= -scale;
      }
    }

    /** Get the 2d keypoints and figure out their 3D position from the depth map
     * @param inputs
     * @param outputs
     * @return
     */
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      cv::Mat out;
      if (*transpose)
      {
        points->copyTo(out);

      }
      else
      {
        cv::Mat(points->t()).copyTo(out);
      }
      out.resize(3, cv::Scalar(0));
      cv::Mat outr;
      cv::Mat(out.t()).copyTo(outr);
      if (outr.depth() == CV_32F)
        convert<float>(float(*scale_factor), outr);
      else
        convert<double>(*scale_factor, outr);
      *points3d = outr;
      return ecto::OK;

    }
    ecto::spore<cv::Mat> points, points3d;
    ecto::spore<double> scale_factor;
    ecto::spore<bool> transpose;
  };

}
using namespace calib;
ECTO_CELL(calib, PointsTo3d, "PointsTo3d", "Converts depth to 3d points.")
