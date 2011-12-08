#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "interfaces.hpp"
#include "hamming.h"
#include <numeric>

/** Cell for ORB feature detection and descriptor extraction
 */
struct SIFT
{
  static void
  declare_params(tendrils& p)
  {
    p.declare<float>("threshold", "");
    p.declare<float>("edgeThreshold", "");
//    p.declare<int>("nOctaves", "", cv::SIFT::CommonParams::DEFAULT_NOCTAVES);
//    p.declare<int>("nOctaveLayers", "", cv::SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS);
//    p.declare<int>("firstOctave", "", cv::SIFT::CommonParams::DEFAULT_FIRST_OCTAVE);
//    p.declare<int>("angleMode", "", cv::SIFT::CommonParams::FIRST_ANGLE);
//
    }

  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    descriptor_extractor_interface::declare_inputs(inputs);
    feature_detector_interface::declare_outputs(outputs);
    descriptor_extractor_interface::declare_outputs(outputs);
  }

  void
  configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    common_params_.nOctaves = params.get<int>("nOctaves");
    common_params_.nOctaveLayers = params.get<int>("nOctaveLayers");
    common_params_.firstOctave = params.get<int>("firstOctave");
    common_params_.angleMode = params.get<int>("angleMode");
    sift_ = cv::SIFT(common_params_);
  }

  int
  process(const tendrils& inputs, const tendrils& outputs)
  {
    std::vector<cv::KeyPoint> keypoints;
    inputs["keypoints"] >> keypoints;
    cv::Mat image, mask;
    inputs["image"] >> image;
    inputs["mask"] >> mask;
    cv::Mat desc;

    sift_(image, mask, keypoints, desc, !keypoints.empty()); //use the provided keypoints if they were given.

    outputs["keypoints"] << keypoints;
    outputs["descriptors"] << desc;

    return ecto::OK;
  }

  cv::SIFT sift_;
  cv::SIFT::CommonParams common_params_;
};

ECTO_CELL(features2d, SIFT, "SIFT",
          "An ORB detector. Takes a image and a mask, and computes keypoints and descriptors(32 byte binary).");
