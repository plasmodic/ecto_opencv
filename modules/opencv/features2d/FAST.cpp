#include <ecto/ecto.hpp>
#include "interfaces.hpp"
struct FAST
{
  static void
  declare_params(tendrils& p)
  {
    p.declare<int>("thresh", "The FAST threshold. 20 is a decent value.", 20);
    p.declare<bool>("nonmax", "Use the FAST nonmax suppression.", true);
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
    thresh_ = params["thresh"];
    nonmax_ = params["nonmax"];
  }

  int
  process(const tendrils& inputs, const tendrils& outputs)
  {
    cv::FastFeatureDetector fd(*thresh_, *nonmax_);

    cv::Mat in, mask;
    inputs["image"] >> in;
    inputs["mask"] >> mask;

    std::vector<cv::KeyPoint> keypoints;
    fd.detect(in, keypoints, mask);
    outputs["keypoints"] << keypoints;
    return 0;
  }
  ecto::spore<int> thresh_;
  ecto::spore<bool> nonmax_;
};
ECTO_CELL(features2d, FAST, "FAST", "Computes fast keypoints given an image, and mask.");
