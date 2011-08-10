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
typedef cv::Mat_<uchar> mask_t;
struct MatchRefinement
{
  static void
  declare_params(tendrils& p)
  {
  }
  static void
  declare_io(const tendrils& p, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<kpts_t>("train", "An input image.");
    inputs.declare<kpts_t>("test", "An mask, same size as image.");
    inputs.declare<matches_t>("matches", "The descriptor matches.");
    outputs.declare<matches_t>("matches", "The verified matches.");
    outputs.declare<cv::Mat>("matches_mask", "The matches mask, same size as the original matches.");

  }

  struct select_train
  {
    int operator()(const cv::DMatch& m)
    {
      return m.trainIdx;
    }
  };
  struct select_test
  {
    int operator()(const cv::DMatch& m)
    {
      return m.queryIdx;
    }
  };

  struct mask_predicate
  {
    mask_predicate(const cv::Mat& mask)
    {
      it = mask.begin<uchar>();
    }
    bool operator()(const cv::DMatch&)
    {
      return 0 == *(it++);

    }
    mask_t::const_iterator it;
  };

  int
  process(const tendrils&inputs, const tendrils& outputs)
  {
    kpts_t train,test;
    matches_t matches;
    inputs["train"] >> train;
    inputs["test"] >> test;
    inputs["matches"] >> matches;

    points_t train_pts,test_pts;
    std::vector<int> train_m,test_m;
    std::transform(matches.begin(),matches.end(),std::back_inserter(train_m), select_train());
    std::transform(matches.begin(),matches.end(),std::back_inserter(test_m), select_test());
    cv::KeyPoint::convert(train,train_pts,train_m);
    cv::KeyPoint::convert(test,test_pts,test_m);
    cv::Mat mask;
    matches_t good_matches;
    if(test_pts.size() > 5 && train_pts.size() > 5)
    {
      cv::Mat H = cv::findHomography(test_pts,train_pts,CV_RANSAC, 10, mask);
      std::remove_copy_if(matches.begin(),matches.end(),std::back_inserter(good_matches),mask_predicate(mask));
    }
    outputs["matches"] << good_matches;
    outputs["matches_mask"] << mask;
    return ecto::OK;
  }
};

ECTO_CELL(features2d, MatchRefinement, "MatchRefinement", "A feature descriptor match refiner.");

