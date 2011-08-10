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
    inputs.declare<kpts_t>("train", "Train keypoints");
    inputs.declare<kpts_t>("test", "Test keypoints.");
    inputs.declare<cv::Mat>("train_image", "Test image.");
    inputs.declare<cv::Mat>("test_image", "Test image.");
    inputs.declare<matches_t>("matches", "The descriptor matches.");
    inputs.declare<cv::Mat>("matches_mask", "The descriptor matches mask.");
    outputs.declare<cv::Mat>("output", "An output image.");
  }

  int
  process(const tendrils& inputs, const tendrils& outputs)
  {
    kpts_t train,test;
    matches_t matches;
    inputs["train"] >> train;
    inputs["test"] >> test;
    inputs["matches"] >> matches;
    cv::Mat test_image,train_image;
    inputs["test_image"] >> test_image;
    inputs["train_image"] >> train_image;
    cv::Mat out_image;
    cv::drawMatches(test_image,test,train_image,train,matches,out_image,cv::Scalar(0,255,0),cv::Scalar::all(-1));
    outputs["output"] << out_image;
    return ecto::OK;
  }
};

ECTO_CELL(features2d, DrawMatches, "DrawMatches", "Draws matches.");

