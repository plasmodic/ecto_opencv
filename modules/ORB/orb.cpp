#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

using ecto::tendrils;

struct ORB: ecto::module_interface
{
  void initialize(tendrils& p)
  {
    p.declare<int> ("n_features", "The number of desired features", 1000);
    p.declare<int> ("n_levels", "The number of scales", 3);
    p.declare<int> ("scale_factor", "The factor between scales", 1.2);
  }

  void config(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    orb_params.first_level_ = 0;
    orb_params.n_levels_ = params.get<int> ("n_levels");
    orb_params.scale_factor_ = params.get<int> ("scale_factor");
    orb = cv::ORB(params.get<int> ("n_features"), orb_params);
    inputs.declare<cv::Mat> ("image", "An input image.");
    inputs.declare<cv::Mat> ("mask", "An mask, same size as image.");

    outputs.declare<std::vector<cv::KeyPoint> > ("kpts", "The keypoints.");
    outputs.declare<cv::Mat> ("descriptors", "The descriptors per keypoints");
  }

  void process(const tendrils& params, const tendrils& inputs,
      tendrils& outputs)
  {
    cv::Mat image = inputs.get<cv::Mat> ("image");
    cv::Mat mask = inputs.get<cv::Mat> ("mask");
    std::vector<cv::KeyPoint>& kpts = outputs.get<std::vector<cv::KeyPoint> > (
        "kpts");
    cv::Mat& descriptors = outputs.get<cv::Mat> ("descriptors");
    orb(image, mask, kpts, descriptors);
  }

  cv::ORB orb;
  cv::ORB::CommonParams orb_params;

};

struct FAST: ecto::module_interface
{
  void config(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    thresh_ = params.get<int> ("thresh");
    outputs.declare<std::vector<cv::KeyPoint> > ("out", "Detected keypoints");
    inputs.declare<cv::Mat> ("image", "The image to detect FAST on.");
    inputs.declare<cv::Mat> ("mask", "optional mask");
  }
  void process(const tendrils& params, const tendrils& inputs,
      tendrils& outputs)
  {
    const cv::Mat& in = inputs.get<cv::Mat> ("image");
    const cv::Mat& mask = inputs.get<cv::Mat> ("mask");
    std::vector<cv::KeyPoint>& kpts = outputs.get<std::vector<cv::KeyPoint> > (
        "out");
    cv::FastFeatureDetector fd(thresh_, true);
    fd.detect(in, kpts, mask);
  }

  void initialize(tendrils& p)
  {
    p.declare<int> ("thresh", "FAST threshhold.", 20);
  }

  int thresh_;
};

struct DrawKeypoints: ecto::module_interface
{
  void config(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("image", "The input image, to draw over.");
    outputs.declare<cv::Mat> ("image", "The output image.");
    inputs.declare<std::vector<cv::KeyPoint> > ("kpts",
        "The keypoints to draw.");
  }
  void process(const tendrils& params, const tendrils& inputs,
      tendrils& outputs)
  {
    const cv::Mat& image = inputs.get<cv::Mat> ("image");
    const std::vector<cv::KeyPoint>& kpts_in = inputs.get<std::vector<
        cv::KeyPoint> > ("kpts");
    cv::Mat& out_image = outputs.get<cv::Mat> ("image");
    cv::drawKeypoints(image, kpts_in, out_image);
  }
};

BOOST_PYTHON_MODULE(orb)
{
  namespace bp = boost::python;
  ecto::wrap<ORB>(
      "ORB",
      "An ORB detector. Takes a image and a mask, and computes keypoints and descriptors(32 byte binary).");
  ecto::wrap<FAST>("FAST", "Computes fast keypoints given an image, and mask.");
  ecto::wrap<DrawKeypoints>("DrawKeypoints");
}
