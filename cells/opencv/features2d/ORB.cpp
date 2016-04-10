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
  declare_params(tendrils& params)
  {
    params.declare(&ORB::n_features_, "n_features", "The number of keypoints to use", 1000);
    params.declare(&ORB::n_levels_, "n_levels", "The number of levels to use for ORB", 3);
    params.declare(&ORB::scale_factor_, "scale_factor", "The scale factor to use for ORB", 1.2);
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
#if CV_MAJOR_VERSION ==3
    orb_ = cv::ORB::create(*n_features_, *scale_factor_, *n_levels_);
#elif (CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION >= 4)
    orb_ = cv::Ptr<cv::ORB>(new cv::ORB(*n_features_, *scale_factor_, *n_levels_));
#else
    cv::ORB::CommonParams orb_params;
    orb_params.first_level_ = 0;
    orb_params.n_levels_ = *n_levels_;
    orb_params.scale_factor_ = *scale_factor_;
    orb_ = cv::Ptr<cv::ORB>(new cv::ORB(*n_features_, orb_params));
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
#if CV_MAJOR_VERSION == 3
    orb_->detectAndCompute(image, mask, keypoints, desc, !keypoints.empty()); //use the provided keypoints if they were given.
#else
    (*orb_)(image, mask, keypoints, desc, !keypoints.empty()); //use the provided keypoints if they were given.
#endif
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

  ecto::spore<int> n_features_, n_levels_;
  ecto::spore<float> scale_factor_;
  cv::Ptr<cv::ORB> orb_;
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
#if CV_MAJOR_VERSION == 3
        size_t distance = cv::norm(desc_i, desc.row(i), cv::NORM_HAMMING);
#else
        size_t distance = cv::normHamming(desc_i.data, desc.row(i).data, desc.cols);
#endif
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
