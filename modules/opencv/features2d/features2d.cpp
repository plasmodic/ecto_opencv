#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

using ecto::tendrils;

struct feature_detector_interface
{
  static void declare_outputs(tendrils& outputs)
  {
    outputs.declare<std::vector<cv::KeyPoint> > ("kpts", "The keypoints.");
  }
  static void declare_io(tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("image", "An input image.");
    inputs.declare<cv::Mat> ("mask", "An mask, same size as image.");
    declare_outputs(outputs);
  }
};

struct feature_extractor_interface
{
  static void declare_io(tendrils& inputs, tendrils& outputs)
  {
    feature_detector_interface::declare_io(inputs,outputs);
    outputs.declare<cv::Mat> ("descriptors", "The descriptors per keypoints");
  }
};

struct ORB
{
  static void declare_params(tendrils& p)
  {
    p.declare<int> ("n_features", "The number of desired features", 1000);
    p.declare<int> ("n_levels", "The number of scales", 3);
    p.declare<float> ("scale_factor", "The factor between scales", 1.2);
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    feature_extractor_interface::declare_io(inputs,outputs);
    inputs.declare<std::vector<cv::KeyPoint> > ("kpts", "Optional kpts.");
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    orb_params.first_level_ = 0;
    orb_params.n_levels_ = params.get<int> ("n_levels");
    orb_params.scale_factor_ = params.get<float> ("scale_factor");
    orb = cv::ORB(params.get<int> ("n_features"), orb_params);
  }

  int process(tendrils& inputs, tendrils& outputs)
  {
    std::vector<cv::KeyPoint> kpts;
    inputs.at("kpts") >> kpts;
    cv::Mat image = inputs.get<cv::Mat> ("image");
    cv::Mat mask = inputs.get<cv::Mat> ("mask");
    cv::Mat desc;
    orb(image,mask,kpts,desc,!kpts.empty());//use the provided kpts if they were given.
		outputs["kpts"] << kpts;
		outputs["descriptors"] << desc;
    return 0;
  }

  cv::ORB orb;
  cv::ORB::CommonParams orb_params;

};

struct FAST
{
  static void declare_params(tendrils& p)
  {
    p.declare<int> ("thresh", "The FAST threshhold. 20 is a decent value.", 20);
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    //use the predefined feature detector inputs, these do not depend on parameters.
    feature_detector_interface::declare_io(inputs,outputs);
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    thresh_ = params.get<int> ("thresh");
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    cv::Mat in = inputs.get<cv::Mat> ("image");
    cv::Mat mask = inputs.get<cv::Mat> ("mask");
    std::vector<cv::KeyPoint> kpts;
    cv::FastFeatureDetector fd(thresh_, true);
    fd.detect(in, kpts, mask);
    outputs["kpts"] << kpts;
    return 0;
  }

  int thresh_;
};

struct DrawKeypoints
{
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("input", "The input image, to draw over.");
    inputs.declare<std::vector<cv::KeyPoint> > ("kpts", "The keypoints to draw.");
    outputs.declare<cv::Mat> ("output", "The output image.");
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    cv::Mat image = inputs.get<cv::Mat> ("input");
    const std::vector<cv::KeyPoint>& kpts_in = inputs.get<std::vector<cv::KeyPoint> > ("kpts");
    cv::Mat& out_image = outputs.get<cv::Mat> ("output");
    cv::drawKeypoints(image, kpts_in, out_image);
    return 0;
  }
};


ECTO_CELL(features2d, ORB, "ORB",
		"An ORB detector. Takes a image and a mask, and computes keypoints and descriptors(32 byte binary).");
ECTO_CELL(features2d, FAST, "FAST",
		"Computes fast keypoints given an image, and mask.");

ECTO_CELL(features2d, DrawKeypoints, "DrawKeypoints",
				"Draws keypoints.");
