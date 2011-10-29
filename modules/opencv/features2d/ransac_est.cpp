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
typedef std::vector<cv::Point3f> points3d_t;
typedef std::vector<cv::DMatch> matches_t;
typedef cv::Mat_<uchar> mask_t;
namespace
{
  struct select_train
  {
    int
    operator()(const cv::DMatch& m) const
    {
      return m.trainIdx;
    }
  };
  struct select_test
  {
    int
    operator()(const cv::DMatch& m) const
    {
      return m.queryIdx;
    }
  };

  struct select_train_3d
  {
    select_train_3d(const cv::Mat& points)
        :
          points(points)
    {
    }

    cv::Point3f
    operator()(const cv::DMatch& m) const
    {
      return points.at<cv::Point3f>(m.trainIdx);
    }
    const cv::Mat& points;
  };
  struct select_test_3d
  {
    select_test_3d(const cv::Mat& points)
        :
          points(points)
    {
    }

    cv::Point3f
    operator()(const cv::DMatch& m) const
    {
      return points.at<cv::Point3f>(m.queryIdx);
    }
    const cv::Mat& points;
  };

  struct nan_predicate
  {
    nan_predicate(const cv::Mat& train, const cv::Mat& test)
        :
          train(train),
          test(test)
    {
    }

    inline bool
    is_nan_safe(const cv::Point3f& p) const
    {
      return p.x != p.x || p.y != p.y || p.z != p.z;
    }

    bool
    operator()(const cv::DMatch& m) const
    {
      return is_nan_safe(train.at<cv::Point3f>(m.trainIdx)) || is_nan_safe(test.at<cv::Point3f>(m.queryIdx));
    }
    const cv::Mat& train;
    const cv::Mat& test;
  };
  template<typename T>
  struct mask_predicate_
  {
    mask_predicate_(T mask)
    {
      it = mask;
    }
    bool
    operator()(const cv::DMatch&)
    {
      return 0 == *(it++);

    }
    T it;
  };
  template<typename T>
  mask_predicate_<T>
  mask_predicate(const T& it)
  {
    return mask_predicate_<T>(it);
  }
}
struct MatchRefinement
{
  static void
  declare_params(tendrils& p)
  {
  }
  static void
  declare_io(const tendrils& p, tendrils& inputs, tendrils& outputs)
  {
    typedef MatchRefinement C;
    inputs.declare(&C::train, "train", "The training kpts.");
    inputs.declare(&C::test, "test", "The test kpts.");
    inputs.declare(&C::matches_in, "matches", "The descriptor matches.");
    outputs.declare(&C::matches_out, "matches", "The verified matches.");
    outputs.declare(&C::matches_mask, "matches_mask", "The matches mask, same size as the original matches.");
    outputs.declare(&C::H_out, "H", "The estimated homography.");
  }

  int
  process(const tendrils&inputs, const tendrils& outputs)
  {
    if(matches_in->empty())
      return ecto::OK;
    points_t train_pts, test_pts;
    std::vector<int> train_m, test_m;
    std::transform(matches_in->begin(), matches_in->end(), std::back_inserter(train_m), select_train());
    std::transform(matches_in->begin(), matches_in->end(), std::back_inserter(test_m), select_test());
    cv::KeyPoint::convert(*train, train_pts, train_m);
    cv::KeyPoint::convert(*test, test_pts, test_m);
    cv::Mat mask;
    matches_t good_matches;
    if (test_pts.size() > 5 && train_pts.size() > 5)
    {
      cv::Mat H = cv::findHomography(test_pts, train_pts, CV_RANSAC, 10, mask);
      std::remove_copy_if(matches_in->begin(), matches_in->end(), std::back_inserter(good_matches),
                          mask_predicate(mask.begin<uchar>()));
      *H_out = H;
    }
    else
    {
      *H_out = cv::Mat();
    }
    *matches_out = *matches_in;
    *matches_mask = mask;
    return ecto::OK;
  }
  ecto::spore<kpts_t> train, test;
  ecto::spore<matches_t> matches_in, matches_out;
  ecto::spore<cv::Mat> matches_mask, H_out;

};

struct MatchRefinement3d
{
  static void
  declare_params(tendrils& p)
  {
  }
  static void
  declare_io(const tendrils& p, tendrils& inputs, tendrils& outputs)
  {
    typedef MatchRefinement3d C;
    inputs.declare(&C::train, "train", "The 3d training points.");
    inputs.declare(&C::test, "test", "The 3d test points.");
    inputs.declare(&C::matches_in, "matches", "The descriptor matches.");
    outputs.declare(&C::matches_out, "matches", "The verified matches.");
    outputs.declare(&C::matches_mask, "matches_mask", "The matches mask, same size as the original matches.");
  }

  int
  process(const tendrils&inputs, const tendrils& outputs)
  {
    matches_t good_matches, non_nan_matches;
    cv::Mat transform(3, 4, CV_64F); //3x4

    std::remove_copy_if(matches_in->begin(), matches_in->end(), std::back_inserter(non_nan_matches),
                        nan_predicate(*train, *test));
    if(non_nan_matches.empty()) return ecto::OK;
    //collate the matches into contiguous blocks of 3d points.
    std::vector<cv::Point3f> train_m, test_m;
    std::transform(non_nan_matches.begin(), non_nan_matches.end(), std::back_inserter(train_m),
                   select_train_3d(*train));
    std::transform(non_nan_matches.begin(), non_nan_matches.end(), std::back_inserter(test_m), select_test_3d(*test));
    cv::Mat train_mat = cv::Mat(train_m), test_mat = cv::Mat(test_m);
    //need to preallocate the mask!
    cv::Mat inliers(1, train_mat.rows, CV_8U);
    //3d 3d estimate
    bool success = cv::estimateAffine3D(train_mat.t(), test_mat.t(), transform, inliers, 0.1, 0.99);
    std::cout << "estimated : " << (success ? std::string("success") : std::string("fail")) << std::endl;
    std::cout << "inliers: " << cv::countNonZero(inliers) << std::endl;
    std::cout << "transform: " << transform << std::endl;

    *matches_out = non_nan_matches;
    *matches_mask = inliers;

    return ecto::OK;
  }
  ecto::spore<cv::Mat> train, test;
  ecto::spore<matches_t> matches_in, matches_out;
  ecto::spore<cv::Mat> matches_mask;

};

ECTO_CELL(features2d, MatchRefinement, "MatchRefinement",
          "A feature descriptor match refiner, using a Homography estimator");
ECTO_CELL(features2d, MatchRefinement3d, "MatchRefinement3d",
          "A feature descriptor match refiner, using an affine 3d to 3d estimator.");

