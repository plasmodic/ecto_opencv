#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "interfaces.h"

/** Cell for ORB feature detection and descriptor extraction
 */
struct SIFT
{
  static void
  declare_params(tendrils& p)
  {
    p.declare<float>("threshold", "");
    p.declare<float>("edgeThreshold", "");
    SIFT_interface::declare_common_params(p);
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
#if (CV_MAJOR_VERSION > 2) || ((CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION >= 4))
    sift_ = cv::SIFT();
    sift_.set("nOctaveLayers", params.get<int>("nOctaveLayers"));
#else
    cv::SIFT::CommonParams common_params;
    common_params.nOctaves = params.get<int>("nOctaves");
    common_params.nOctaveLayers = params.get<int>("nOctaveLayers");
    common_params.firstOctave = params.get<int>("firstOctave");
    common_params.angleMode = params.get<int>("angleMode");
    sift_ = cv::SIFT(common_params);
#endif
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
};

ECTO_CELL(features2d, SIFT, "SIFT",
          "An ORB detector. Takes a image and a mask, and computes keypoints and descriptors(32 byte binary).");
