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

#if CV_MAJOR_VERSION == 3
#include <opencv2/xfeatures2d.hpp>
#endif

#include "interfaces.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** This is an enum on the different supported types */
enum DescriptorExtractorType
{
  SIFT, SURF, ORB, BRIEF
};

const char* descriptor_extractor_type_names_tmp[] =
{ "SIFT", "SURF", "ORB", "BRIEF" };
/** This is the corresponding list of OpenCV names for each Descriptor type */
const std::vector<std::string> descriptor_extractor_type_names(descriptor_extractor_type_names_tmp,
                                                               descriptor_extractor_type_names_tmp + 4);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Cell for ORB feature detection and descriptor extraction
 */
template<DescriptorExtractorType T>
struct EctoDescriptorExtractor
{
  static void
  declare_params(tendrils& p)
  {
    throw std::runtime_error("Unsupported DescriptorExtractor type");
  }

  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    descriptor_extractor_interface::declare_inputs(outputs);
    descriptor_extractor_interface::declare_outputs(outputs);
  }

  void
  configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
#if CV_MAJOR_VERSION == 3
    switch(T) {
      case SIFT:
        descriptor_extractor_ = cv::xfeatures2d::SIFT::create();
        break;
      case SURF:
        descriptor_extractor_ = cv::xfeatures2d::SURF::create();
        break;
      case ORB:
        descriptor_extractor_ = cv::ORB::create();
        break;
      case BRIEF:
        descriptor_extractor_ = cv::xfeatures2d::BriefDescriptorExtractor::create();
        break;
    }
#else
    descriptor_extractor_ = cv::DescriptorExtractor::create(descriptor_extractor_type_names[T]);
#endif
    read_tendrils_as_file_node(params, descriptor_extractor_);
  }

  int
  process(const tendrils& inputs, const tendrils& outputs)
  {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat image, descriptors;
    inputs["image"] >> image;

    descriptor_extractor_->compute(image, keypoints, descriptors);

    outputs["descriptors"] << descriptors;
    return ecto::OK;
  }

  cv::Ptr<cv::DescriptorExtractor> descriptor_extractor_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<>
void
EctoDescriptorExtractor<ORB>::declare_params(tendrils& p)
{
}

ECTO_CELL(features2d, EctoDescriptorExtractor<ORB>, "ORBDescriptor", "An ORB descriptor extractor.");
