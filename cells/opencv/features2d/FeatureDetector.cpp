/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <stdio.h>

#include <ecto/ecto.hpp>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#if CV_MAJOR_VERSION == 3
#include <opencv2/xfeatures2d.hpp>
#endif

#include "interfaces.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** This is an enum on the different supported types */
enum FeatureDetectorType
{
  FAST, ORB, SIFT
};

const char* feature_detector_type_names_tmp[] =
{ "FAST", "ORB", "SIFT" };
/** This is the corresponding list of OpenCV names for each Detector type */
const std::vector<std::string> feature_detector_type_names(feature_detector_type_names_tmp,
                                                           feature_detector_type_names_tmp + 3);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Cell for ORB feature detection and descriptor extraction
 */
template<FeatureDetectorType T>
struct EctoFeatureDetector
{
  static void
  declare_params(tendrils& p)
  {
    throw std::runtime_error("Unsupported FeatureDetector type");
  }

  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    feature_detector_interface::declare_inputs(outputs);
    feature_detector_interface::declare_outputs(outputs);
  }

  void
  configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
#if CV_MAJOR_VERSION == 3
    switch(T) {
      case FAST:
        feature_detector_ = cv::FastFeatureDetector::create();
        break;
      case ORB:
        feature_detector_ = cv::ORB::create();
        break;
      case SIFT:
        feature_detector_ = cv::xfeatures2d::SIFT::create();
        break;
    }
#else
    feature_detector_ = cv::FeatureDetector::create(feature_detector_type_names[T]);
#endif
    read_tendrils_as_file_node(params, feature_detector_);
  }

  int
  process(const tendrils& inputs, const tendrils& outputs)
  {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat image, mask;
    inputs["image"] >> image;
    inputs["mask"] >> mask;

    feature_detector_->detect(image, keypoints, mask);

    outputs["keypoints"] << keypoints;
    return ecto::OK;
  }

  cv::Ptr<cv::FeatureDetector> feature_detector_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<>
void
EctoFeatureDetector<FAST>::declare_params(tendrils& p)
{
  p.declare<int>("thresh", "The FAST threshold. 20 is a decent value.", 20);
  p.declare<bool>("nonmax", "Use the FAST nonmax suppression.", true);
}

ECTO_CELL(features2d, EctoFeatureDetector<FAST>, "FASTFeature", "A FAST feature detector.");

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<>
void
EctoFeatureDetector<ORB>::declare_params(tendrils& p)
{
  p.declare<int>("n_features", "The number of desired features", 1000);
  p.declare<int>("n_levels", "The number of scales", 3);
  p.declare<int>("first_level", "The first level of the scales", 0);
  p.declare<float>("scale_factor", "The factor between scales", 1.2);
}

ECTO_CELL(features2d, EctoFeatureDetector<ORB>, "ORBFeature", "An ORB feature detector.");
