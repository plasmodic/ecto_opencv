#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
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

/** Cell for ORB feature detection and descriptor extraction
 */
struct ORB
{
  static void
  declare_params(tendrils& p)
  {
    p.declare<int>("n_features", "The number of desired features", 1000);
    p.declare<int>("n_levels", "The number of scales", 3);
    p.declare<float>("scale_factor", "The factor between scales", 1.2);
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
    orb_params_.first_level_ = 0;
    orb_params_.n_levels_ = params.get<int>("n_levels");
    orb_params_.scale_factor_ = params.get<float>("scale_factor");
    orb_ = cv::ORB(params.get<int>("n_features"), orb_params_);
  }

  int
  process(const tendrils& inputs, const tendrils& outputs)
  {
    std::vector<cv::KeyPoint> keypoints;
    inputs["keypoints"] >> keypoints;
    cv::Mat image = inputs.get<cv::Mat>("image");
    cv::Mat mask = inputs.get<cv::Mat>("mask");
    cv::Mat desc;
    orb_(image, mask, keypoints, desc, !keypoints.empty()); //use the provided keypoints if they were given.
    outputs["keypoints"] << keypoints;
    outputs["descriptors"] << desc;
    return 0;
  }

  cv::ORB orb_;
  cv::ORB::CommonParams orb_params_;
};

struct FAST
{
  static void
  declare_params(tendrils& p)
  {
    p.declare<int>("thresh", "The FAST threshold. 20 is a decent value.", 20);
  }

  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    //use the predefined feature detector inputs, these do not depend on parameters.
    feature_detector_interface::declare_inputs(inputs);
    feature_detector_interface::declare_outputs(outputs);
  }

  void
  configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    thresh_ = params.get<int>("thresh");
  }

  int
  process(const tendrils& inputs, const tendrils& outputs)
  {
    cv::Mat in = inputs.get<cv::Mat>("image");
    cv::Mat mask = inputs.get<cv::Mat>("mask");
    std::vector<cv::KeyPoint> keypoints;
    cv::FastFeatureDetector fd(thresh_, true);
    fd.detect(in, keypoints, mask);
    outputs["keypoints"] << keypoints;
    return 0;
  }

  int thresh_;
};

/** Interface to cv::drawKeypoints, to draw keypoints to an image
 */
struct DrawKeypoints
{
  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat>("image", "The input image, to draw over.");
    inputs.declare<std::vector<cv::KeyPoint> >("keypoints", "The keypoints to draw.");
    outputs.declare<cv::Mat>("image", "The output image.");
  }

  int
  process(const tendrils& inputs, const tendrils& outputs)
  {
    cv::Mat image = inputs.get<cv::Mat>("image");
    const std::vector<cv::KeyPoint>& keypoints_in = inputs.get<std::vector<cv::KeyPoint> >("keypoints");
    cv::Mat& out_image = outputs.get<cv::Mat>("image");
    cv::drawKeypoints(image, keypoints_in, out_image);
    return 0;
  }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ECTO_CELL(features2d, ORB, "ORB",
          "An ORB detector. Takes a image and a mask, and computes keypoints and descriptors(32 byte binary).");
ECTO_CELL(features2d, FAST, "FAST", "Computes fast keypoints given an image, and mask.");

ECTO_CELL(features2d, DrawKeypoints, "DrawKeypoints", "Draws keypoints.");
