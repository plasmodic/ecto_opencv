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

#include <boost/foreach.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

using ecto::tendrils;

namespace
{
  /** Convert the DMatch struct into a cv::Mat for ease of use with numpy and the like.
   */
  struct MatchesToMat
  {
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<std::vector<cv::DMatch> >("matches", "The matches");
      outputs.declare<cv::Mat>("matches", "An nx3 matrix of matched indices with distances in the 3rd column");
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      const std::vector<cv::DMatch> & matches = inputs.get<std::vector<cv::DMatch> >("matches");
      cv::Mat_<double> out_matches(matches.size(), 3);

      // Copy the x,y only
      size_t i = 0;
      BOOST_FOREACH(const cv::DMatch & m, matches)
          {
            out_matches(i, 0) = m.trainIdx;
            out_matches(i, 1) = m.queryIdx;
            out_matches(i, 2) = m.distance;
            ++i;
          }
      outputs["matches"] << cv::Mat(out_matches);
      return 0;
    }
  };
}

ECTO_CELL(features2d, MatchesToMat, "MatchesToMat", "Takes matches and turns them into a cv mat alias..");
