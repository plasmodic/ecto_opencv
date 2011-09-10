#pragma once
#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

using ecto::tendrils;

/** Interface to feature detection, reused by specific feature detectors
 */
struct feature_detector_interface
{
  static void
  declare_outputs(tendrils& outputs)
  {
    outputs.declare<std::vector<cv::KeyPoint> >("keypoints", "The keypoints.");
  }
  static void
  declare_inputs(tendrils& inputs)
  {
    inputs.declare<cv::Mat>("image", "An input image.");
    inputs.declare<cv::Mat>("mask", "An mask, same size as image.");
  }
};


/** Interface to descriptor extraction, reused by specific descriptor extractors
 */
struct descriptor_extractor_interface
{
  static void
  declare_outputs(tendrils& outputs)
  {
    outputs.declare<cv::Mat>("descriptors", "The descriptors per keypoints");
  }
  static void
  declare_inputs(tendrils& inputs)
  {
    inputs.declare<cv::Mat>("image", "An input image.");
    inputs.declare<cv::Mat>("mask", "An mask, same size as image.");
    inputs.declare<std::vector<cv::KeyPoint> >("keypoints", "The keypoints.");
  }
};
