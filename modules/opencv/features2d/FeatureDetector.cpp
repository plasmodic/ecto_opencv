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

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "interfaces.hpp"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::string
temporary_yml_file_name()
{
  std::string fname;
  {
    char buffer[L_tmpnam];
    char* p = std::tmpnam(buffer);
    if (p != NULL)
    {
      fname = std::string(buffer) + ".yml";
    }
    else
      throw std::runtime_error("Could not create temporary filename!");
  }
  return fname;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

enum FeatureDetectorType
{
  ORB, FAST
};

const char* feature_detector_type_names_tmp[] =
{ "ORB", "FAST" };
const std::vector<std::string> feature_detector_type_names(feature_detector_type_names_tmp,
                                                           feature_detector_type_names_tmp + 1);

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
    feature_detector_ = cv::FeatureDetector::create(feature_detector_type_names[T]);

    // Get the binary file
    std::string file_name = temporary_yml_file_name();

    // Read it
    {
      cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
      BOOST_FOREACH(const ecto::tendrils::value_type &tendril_pair, params)
          {
            std::string tendril_name = tendril_pair.first;
            const ecto::tendril & tendril = *(tendril_pair.second);
            std::string type_name = tendril.type_name();
            fs << tendril_name;
            if (type_name == "int")
              fs << tendril.get<int>();
            else if (type_name == "float")
              fs << tendril.get<float>();
            else
            {
              std::string error_message = "Unsupported type: ";
              error_message += type_name;
              throw std::runtime_error(error_message);
            }
          }
    }

    cv::FileStorage fs(file_name, cv::FileStorage::READ);
    feature_detector_->read(fs.root());

    boost::filesystem::remove(file_name.c_str());
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
EctoFeatureDetector<ORB>::declare_params(tendrils& p)
{
  p.declare<int>("n_features", "The number of desired features", 1000);
  p.declare<int>("n_levels", "The number of scales", 3);
  p.declare<int>("first_level", "The first level of the scales", 0);
  p.declare<float>("scale_factor", "The factor between scales", 1.2);
}

ECTO_CELL(features2d, EctoFeatureDetector<ORB>, "ORBFeature", "An ORB feature detector.");

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<>
void
EctoFeatureDetector<FAST>::declare_params(tendrils& p)
{
  p.declare<int>("thresh", "The FAST threshold. 20 is a decent value.", 20);
  p.declare<bool>("nonmax", "Use the FAST nonmax suppression.", true);
}

ECTO_CELL(features2d, EctoFeatureDetector<FAST>, "FAST", "A FAST feature detector.");
