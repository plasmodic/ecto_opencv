#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "interfaces.hpp"

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
    cv::Mat image, mask;
    inputs["image"] >> image;
    inputs["mask"] >> mask;
    cv::Mat desc;
    orb_(image, mask, keypoints, desc, !keypoints.empty()); //use the provided keypoints if they were given.
    if (!mask.empty())
    {
      //need to do keypoint validation as ORB is broken.
      cv::Mat good_desc;
      std::vector<cv::KeyPoint> good_keypoints;
      good_keypoints.reserve(keypoints.size());
      good_desc.reserve(32 * keypoints.size());
      for (int i = 0, end = keypoints.size(); i < end; ++i)
      {
        const cv::Point2f& p2d = keypoints[i].pt;
        int u = p2d.x + 0.5f;
        int v = p2d.y + 0.5f;
        if (mask.at<uchar>(v, u))
        {
          good_keypoints.push_back(keypoints[i]);
          good_desc.push_back(desc.row(i));
        }
      }
//      std::cout << "points diff : " << int(keypoints.size()) - int(good_keypoints.size()) << std::endl;
      outputs["keypoints"] << good_keypoints;
      outputs["descriptors"] << good_desc;
    }
    else
    {
      outputs["keypoints"] << keypoints;
      outputs["descriptors"] << desc;
    }
    return ecto::OK;
  }

  cv::ORB orb_;
  cv::ORB::CommonParams orb_params_;
};

ECTO_CELL(features2d, ORB, "ORB",
          "An ORB detector. Takes a image and a mask, and computes keypoints and descriptors(32 byte binary).");
