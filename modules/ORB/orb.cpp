#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

using ecto::tendrils;

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
    inputs.declare<cv::Mat> ("image", "An input image.");
    inputs.declare<cv::Mat> ("mask", "An mask, same size as image.");

    outputs.declare<std::vector<cv::KeyPoint> > ("kpts", "The keypoints.");
    outputs.declare<cv::Mat> ("descriptors", "The descriptors per keypoints");
  }

  void config(tendrils& params)
  {
    orb_params.first_level_ = 0;
    orb_params.n_levels_ = params.get<int> ("n_levels");
    orb_params.scale_factor_ = params.get<int> ("scale_factor");
    orb = cv::ORB(params.get<int> ("n_features"), orb_params);
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    cv::Mat image = inputs.get<cv::Mat> ("image");
    cv::Mat mask = inputs.get<cv::Mat> ("mask");
    std::vector<cv::KeyPoint>& kpts = outputs.get<std::vector<cv::KeyPoint> > ("kpts");
    cv::Mat& descriptors = outputs.get<cv::Mat> ("descriptors");
    orb(image, mask, kpts, descriptors);
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
    outputs.declare<std::vector<cv::KeyPoint> > ("out", "Detected keypoints");
    inputs.declare<cv::Mat> ("image", "The image to detect FAST on.");
    inputs.declare<cv::Mat> ("mask", "optional mask");
  }

  void config(tendrils& params)
  {
    thresh_ = params.get<int> ("thresh");
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    cv::Mat in = inputs.get<cv::Mat> ("image");
    cv::Mat mask = inputs.get<cv::Mat> ("mask");
    std::vector<cv::KeyPoint>& kpts = outputs.get<std::vector<cv::KeyPoint> > ("out");
    cv::FastFeatureDetector fd(thresh_, true);
    fd.detect(in, kpts, mask);
    return 0;
  }

  int thresh_;
};

struct DrawKeypoints
{
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("image", "The input image, to draw over.");
    outputs.declare<cv::Mat> ("image", "The output image.");
    inputs.declare<std::vector<cv::KeyPoint> > ("kpts", "The keypoints to draw.");
  }
  int process(const tendrils& inputs, tendrils& outputs)
  {
    cv::Mat image = inputs.get<cv::Mat> ("image");
    const std::vector<cv::KeyPoint>& kpts_in = inputs.get<std::vector<cv::KeyPoint> > ("kpts");
    cv::Mat& out_image = outputs.get<cv::Mat> ("image");
    cv::drawKeypoints(image, kpts_in, out_image);
    return 0;
  }
};

BOOST_PYTHON_MODULE(orb)
{
  namespace bp = boost::python;
  ecto::wrap<ORB>("ORB",
      "An ORB detector. Takes a image and a mask, and computes keypoints and descriptors(32 byte binary).");
  ecto::wrap<FAST>("FAST", "Computes fast keypoints given an image, and mask.");
  ecto::wrap<DrawKeypoints>("DrawKeypoints");
}
