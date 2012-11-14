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

/** This is an implementation of a fast plane detection */

#include <list>
#include <numeric>
#include <set>
#include <string>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/rgbd/rgbd.hpp>

#include <ecto/ecto.hpp>

#include <opencv_creative/reader.h>

using ecto::tendrils;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct CreativeReader
{
  static void
  declare_params(tendrils& params)
  {
  }

  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    outputs.declare(&CreativeReader::image_, "image", "The current image.");
    outputs.declare(&CreativeReader::depth_, "depth_raw", "The current depth frame.");
    outputs.declare(&CreativeReader::points3d_, "points3d", "The current depth frame.");
    outputs.declare(&CreativeReader::K_, "K", "The calibration matrix");
  }

  void
  configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    creative::Reader::setImageTypes(creative::Reader::COLOR + creative::Reader::DEPTH + creative::Reader::POINTS3D);
    creative::Reader::initialize();
  }

  int
  process(const tendrils& inputs, const tendrils& outputs)
  {
    std::vector<cv::Mat> images;
    creative::Reader::getImages(images);
    images[0].copyTo(*image_);
    images[1].copyTo(*depth_);
    images[2].copyTo(*points3d_);

    return ecto::OK;
  }

private:
  /** Input 3d points */
  ecto::spore<cv::Mat> points3d_;
  ecto::spore<cv::Mat> image_;
  ecto::spore<cv::Mat> depth_;
  ecto::spore<cv::Mat> K_;
};

ECTO_CELL(rgbd, CreativeReader, "CreativeReader", "The image reader for the depth sensot from creative.")
