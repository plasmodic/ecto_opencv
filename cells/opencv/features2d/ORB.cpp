#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "interfaces.h"
#include <numeric>

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
#if (CV_MAJOR_VERSION > 2) || ((CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION >= 4))
    orb_ = cv::ORB(params.get<int>("n_features"), params.get<float>("scale_factor"), params.get<int>("n_levels"));
#else
    cv::ORB::CommonParams orb_params;
    orb_params.first_level_ = 0;
    orb_params.n_levels_ = params.get<int>("n_levels");
    orb_params.scale_factor_ = params.get<float>("scale_factor");
    orb_ = cv::ORB(params.get<int>("n_features"), orb_params);
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
};
struct DescriptorAccumulator
{
  typedef DescriptorAccumulator C;

  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare(&C::in_descriptors, "descriptors", "The input descriptors.");
    outputs.declare(&C::out_descriptors, "descriptors", "A cumulative view of all descriptors.");
  }

  int
  process(const tendrils& inputs, const tendrils& outputs)
  {
    cv::Mat desc;
    in_descriptors->copyTo(desc);
    if (cumulative_desc_.empty())
    {
      cumulative_desc_ = desc;
      return ecto::OK;
    }
    cumulative_desc_.push_back(desc);
    cumulative_desc_.copyTo(*out_descriptors);
    return ecto::OK;
  }
  cv::Mat cumulative_desc_;
  ecto::spore<cv::Mat> in_descriptors;
  ecto::spore<cv::Mat> out_descriptors;
};
struct ORBstats
{
  typedef ORBstats C;

  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare(&C::in_descriptors, "descriptors", "The input descriptors.");
    outputs.declare(&C::out_hist, "distances", "A histogram of the distances in this set.");

  }

  int
  process(const tendrils& inputs, const tendrils& outputs)
  {
    std::vector<int> distances(256, 0);
    cv::Mat desc;
    in_descriptors->copyTo(desc);
    while (desc.rows)
    {
      cv::Mat desc_i;
      desc.row(desc.rows - 1).copyTo(desc_i);
      desc.pop_back(1);
      for (int i = 0, end = desc.rows; i < end; i++)
      {
        size_t distance = cv::normHamming(desc_i.data, desc.row(i).data, desc.cols);
        distances[distance]++;
      }
    }
    *out_hist = cv::Mat(distances).clone();
    return ecto::OK;
  }
  ecto::spore<cv::Mat> in_descriptors;
  ecto::spore<cv::Mat> out_hist;
};
ECTO_CELL(features2d, ORB, "ORB",
          "An ORB detector. Takes a image and a mask, and computes keypoints and descriptors(32 byte binary).");
ECTO_CELL(features2d, ORBstats, "ORBstats", "Prints stats on ORB descriptors.");
ECTO_CELL(features2d, DescriptorAccumulator, "DescriptorAccumulator", "Accumulates descriptors.");
