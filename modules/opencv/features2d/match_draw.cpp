#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <algorithm>

using ecto::tendrils;

typedef std::vector<cv::KeyPoint> kpts_t;
typedef std::vector<cv::Point2f> points_t;
typedef std::vector<cv::DMatch> matches_t;
struct DrawMatches
{
  static void
  declare_params(tendrils& p)
  {
  }
  static void
  declare_io(const tendrils& p, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat>("train", "Train keypoints");
    inputs.declare<cv::Mat>("test", "Test keypoints.");
    inputs.declare<cv::Mat>("train_image", "Test image.");
    inputs.declare<cv::Mat>("test_image", "Test image.");
    inputs.declare<matches_t>("matches", "The descriptor matches.");
    inputs.declare<cv::Mat>("matches_mask", "The descriptor matches mask.");
    outputs.declare<cv::Mat>("output", "An output image.");
  }

  int
  process(const tendrils& inputs, const tendrils& outputs)
  {
    cv::Mat train, test;
    matches_t matches;
    inputs["train"] >> train;
    inputs["test"] >> test;
    if (test.empty() || train.empty())
      return ecto::OK;
    inputs["matches"] >> matches;
    cv::Mat test_image, train_image;
    inputs["test_image"] >> test_image;
    inputs["train_image"] >> train_image;
    cv::Mat matches_mask;
    inputs["matches_mask"] >> matches_mask;
    cv::Mat out_image;
    kpts_t train_kpts, test_kpts;
    std::vector<cv::Point2f> train_pts, test_pts;
    train = train.reshape(2,0);
    test = test.reshape(2,0);

    std::copy(train.begin<cv::Point2f>(), train.end<cv::Point2f>(), std::back_inserter(train_pts));
    std::copy(test.begin<cv::Point2f>(), test.end<cv::Point2f>(), std::back_inserter(test_pts));
    cv::KeyPoint::convert(train_pts, train_kpts);
    cv::KeyPoint::convert(test_pts, test_kpts);
//    std::cout << "matches:" << matches.size() << std::endl;
//    std::cout << "test:" << test_pts.size() << std::endl;
//    std::cout << "train:" << train_pts.size() << std::endl;
//    std::cout << test_image.cols << ":" << test_image.rows << std::endl;
//    std::cout << train_image.cols << ":" << train_image.rows << std::endl;
//
    cv::drawMatches(test_image, test_kpts, train_image, train_kpts, matches, out_image, cv::Scalar(0, 255, 0),
                    cv::Scalar(0, 0, 255), matches_mask);
    outputs["output"] << out_image;
    return ecto::OK;
  }
};

ECTO_CELL(features2d, DrawMatches, "DrawMatches", "Draws matches.");

