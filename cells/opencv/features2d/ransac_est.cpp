#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <numeric>
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

  template<typename PointT>
  struct select_train_
  {
    select_train_(const cv::Mat& points)
        :
          points(points)
    {
    }

    PointT
    operator()(const cv::DMatch& m) const
    {
      return points.at<PointT>(m.trainIdx);
    }
    const cv::Mat& points;
  };
  template<typename PointT>
  struct select_test_
  {
    select_test_(const cv::Mat& points)
        :
          points(points)
    {
    }

    PointT
    operator()(const cv::DMatch& m) const
    {
      PointT x = points.at<PointT>(m.queryIdx);
      return x;
    }
    const cv::Mat& points;
  };

  template<typename PointT>
  struct subtract_
  {
    subtract_(const PointT& p)
        :
          p(p)
    {
    }

    PointT
    operator()(const PointT& x) const
    {
      return x - p;
    }
    const PointT& p;
  };
  typedef select_test_<cv::Point3f> select_test_3d;
  typedef select_test_<cv::Point2f> select_test_2d;
  typedef select_train_<cv::Point3f> select_train_3d;
  typedef select_train_<cv::Point2f> select_train_2d;

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

  struct match_distance_predicate_
  {
    match_distance_predicate_(double dist)
        :
          dist(dist)
    {
    }
    bool
    operator()(const cv::DMatch& m) const
    {
      return m.distance > dist;
    }
    double dist;
  };

  template<int dist>
  struct match_distance_predicate
  {

    bool
    operator()(const cv::DMatch& m) const
    {
      return m.distance > dist;
    }
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

  cv::Point3f
  demean_points(const std::vector<cv::Point3f>& points, std::vector<cv::Point3f>& out)
  {
    cv::Point3f p;
    p = std::accumulate(points.begin(), points.end(), p);
    p *= 1.f / points.size();
    cv::Mat tm(points);
    out.resize(points.size());
    std::transform(points.begin(), points.end(), out.begin(), subtract_<cv::Point3f>(p));
    return p;
  }

  template<typename T>
  int
  sign(T f)
  {
    if (f > 0)
      return 1;
    else
      return -1;
  }

}
struct MatchRefinement
{
  typedef MatchRefinement C;

  static void
  declare_params(tendrils& p)
  {
    p.declare(&C::match_distance, "match_distance", "The match distance threshhold.", 120);
  }
  static void
  declare_io(const tendrils& p, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare(&C::train, "train", "The training points.");
    inputs.declare(&C::test, "test", "The test points.");
    inputs.declare(&C::matches_in, "matches", "The descriptor matches.");
    outputs.declare(&C::matches_out, "matches", "The verified matches.");
    outputs.declare(&C::matches_mask, "matches_mask", "The matches mask, same size as the original matches.");
    outputs.declare(&C::H_out, "H", "The estimated homography.");
  }

  int
  process(const tendrils&inputs, const tendrils& outputs)
  {

    if (matches_in->empty())
      return ecto::OK;
    matches_t good_matches;
    std::remove_copy_if(matches_in->begin(), matches_in->end(), std::back_inserter(good_matches),
                        match_distance_predicate_(*match_distance));

    points_t train_pts, test_pts;
    std::transform(good_matches.begin(), good_matches.end(), std::back_inserter(train_pts), select_train_2d(*train));
    std::transform(good_matches.begin(), good_matches.end(), std::back_inserter(test_pts), select_test_2d(*test));
    cv::Mat mask;
    if (test_pts.size() > 5 && train_pts.size() > 5)
    {
      cv::Mat H = cv::findHomography(test_pts, train_pts, CV_RANSAC, 10, mask);
      *H_out = H;
    }
    else
    {
      *H_out = cv::Mat();
    }
    *matches_out = good_matches;
    *matches_mask = mask;
    return ecto::OK;
  }
  ecto::spore<cv::Mat> train, test;
  ecto::spore<matches_t> matches_in, matches_out;
  ecto::spore<cv::Mat> matches_mask, H_out;
  ecto::spore<double> match_distance;

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
    outputs.declare(&C::R_out, "R");
    outputs.declare(&C::T_out, "T");
  }

  int
  process(const tendrils&inputs, const tendrils& outputs)
  {
    matches_t good_matches, non_nan_matches;
    cv::Mat transform(3, 4, CV_64F); //3x4

    std::remove_copy_if(matches_in->begin(), matches_in->end(), std::back_inserter(non_nan_matches),
                        nan_predicate(*train, *test));

    //    std::cout << "Non-NAN matches: " << non_nan_matches.size() << std::endl;

    if (non_nan_matches.empty())
      return ecto::OK;

    //need to preallocate the mask!
    //    cv::Mat inl(1, non_nan_matches.size(), CV_8U, 1);
    //    *matches_out = non_nan_matches;
    //    *matches_mask = inl;

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

    cv::Mat R, T;
    std::cout << "R: " << transform.colRange(0, 3) << std::endl;
    transform.colRange(0, 3).copyTo(R);
    std::cout << "T: " << transform.colRange(3, 4) << std::endl;
    transform.colRange(3, 4).copyTo(T);
    *R_out = R;
    *T_out = T;
    *matches_out = non_nan_matches;
    *matches_mask = inliers;

    return ecto::OK;
  }
  ecto::spore<cv::Mat> train, test, R_out, T_out;
  ecto::spore<matches_t> matches_in, matches_out;
  ecto::spore<cv::Mat> matches_mask;

};

struct MatchRefinementPnP
{
  typedef MatchRefinementPnP C;

  static void
  declare_params(tendrils& p)
  {
    p.declare(&C::n_iters, "n_iters", "number of ransac iterations", 100);
    p.declare(&C::reprojection_error, "reprojection_error", "error threshold", 8);
    p.declare(&C::min_inliers, "min_inliers", "minimum number of inliers", 100);
    p.declare(&C::inlier_thresh, "inlier_thresh", "The thresh hold on number of inliers to consider pose found.", 30);

  }
  static void
  declare_io(const tendrils& p, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare(&C::K, "K", "Camera model.");
    inputs.declare(&C::train, "train", "The 3d training points.");
    inputs.declare(&C::test, "test", "The 3d test points.");
    inputs.declare(&C::matches_in, "matches", "The descriptor matches.");
    outputs.declare(&C::matches_out, "matches", "The verified matches.");
    outputs.declare(&C::matches_mask, "matches_mask", "The matches mask, same size as the original matches.");
    outputs.declare(&C::R_out, "R");
    outputs.declare(&C::T_out, "T");
    outputs.declare(&C::found_out, "found");
  }

  int
  process(const tendrils&inputs, const tendrils& outputs)
  {
    *found_out = false;
    matches_t good_matches;
    std::remove_copy_if(matches_in->begin(), matches_in->end(), std::back_inserter(good_matches),
                        match_distance_predicate<50>());

    if (good_matches.size() < 50)
      return ecto::OK;

    //collate the matches into contiguous blocks of 3d points.
    points3d_t train_m;
    points_t test_m;
    std::transform(good_matches.begin(), good_matches.end(), std::back_inserter(train_m), select_train_3d(*train));
    std::transform(good_matches.begin(), good_matches.end(), std::back_inserter(test_m), select_test_2d(*test));
    //need to preallocate the mask!
    cv::Mat rvec, tvec;
    std::vector<int> inliers;
    //3d 3d estimate
    cv::solvePnPRansac(train_m, test_m, *K, cv::Mat::zeros(1, 4, K->type()), rvec, tvec, false, *n_iters,
                       *reprojection_error, *min_inliers, inliers);
    cv::Mat inlier_mask = cv::Mat::ones(good_matches.size(), 1, CV_8UC1);
    for (int i = 0, end = inliers.size(); i < end; i++)
    {
      int idx = inliers[i];
      inlier_mask.at<uchar>(idx) = 255;
    }
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    *R_out = R;
    *T_out = tvec;
    *matches_out = good_matches;
    *matches_mask = inlier_mask;
    *found_out = inliers.size() > *inlier_thresh;
    return ecto::OK;
  }
  ecto::spore<cv::Mat> K, train, test, R_out, T_out;
  ecto::spore<matches_t> matches_in, matches_out;
  ecto::spore<cv::Mat> matches_mask;
  ecto::spore<bool> found_out;
  ecto::spore<unsigned> n_iters, min_inliers;
  ecto::spore<float> reprojection_error, inlier_thresh;

};

struct MatchRefinementHSvd
{
  typedef MatchRefinementHSvd C;

  static void
  declare_params(tendrils& p)
  {
    p.declare(&C::n_iters, "n_iters", "number of ransac iterations", 200);
    p.declare(&C::reprojection_error, "reprojection_error", "error threshold", 43.5);
    p.declare(&C::min_inliers, "min_inliers", "minimum number of inliers", 100);
    p.declare(&C::inlier_thresh, "inlier_thresh", "The inlier threshold of pose found.", 25);
  }
  static void
  declare_io(const tendrils& p, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare(&C::train_2d, "train_2d", "The 2d training points.");
    inputs.declare(&C::test_2d, "test_2d", "The 2d test points.");
    inputs.declare(&C::train_3d, "train_3d", "The 3d training points.");
    inputs.declare(&C::test_3d, "test_3d", "The 3d test points.");
    inputs.declare(&C::matches_in, "matches", "The descriptor matches.");
    outputs.declare(&C::matches_out, "matches", "The verified matches.");
    outputs.declare(&C::matches_mask, "matches_mask", "The matches mask, same size as the original matches.");
    outputs.declare(&C::R_out, "R");
    outputs.declare(&C::T_out, "T");
    outputs.declare(&C::found_out, "found");
  }

  int
  process(const tendrils&inputs, const tendrils& outputs)
  {
    *found_out = false;
    matches_t good_matches, good_matches_H;
    std::remove_copy_if(matches_in->begin(), matches_in->end(), std::back_inserter(good_matches),
                        match_distance_predicate<90>());
    if (good_matches.size() < 2 * (*min_inliers))
      return ecto::OK;

    //collate the matches into contiguous blocks of 3d points.
    points_t train_pts;
    points_t test_pts;
    std::transform(good_matches.begin(), good_matches.end(), std::back_inserter(train_pts), select_train_2d(*train_2d));
    std::transform(good_matches.begin(), good_matches.end(), std::back_inserter(test_pts), select_test_2d(*test_2d));
    //need to preallocate the mask!
    cv::Mat rvec, tvec;
    cv::Mat inlier_mask;
    cv::Mat H = cv::findHomography(test_pts, train_pts, CV_RANSAC, *reprojection_error, inlier_mask);
    std::remove_copy_if(good_matches.begin(), good_matches.end(), std::back_inserter(good_matches_H),
                        mask_predicate(inlier_mask.begin<uchar>()));

    points3d_t train_pts_3d, demeaned_train_pts;
    points3d_t test_pts_3d, demeaned_test_pts;
    std::transform(good_matches_H.begin(), good_matches_H.end(), std::back_inserter(train_pts_3d),
                   select_train_3d(*train_3d));
    std::transform(good_matches_H.begin(), good_matches_H.end(), std::back_inserter(test_pts_3d),
                   select_test_3d(*test_3d));

    cv::Point3f train_centroid = demean_points(train_pts_3d, demeaned_train_pts);
    cv::Point3f test_centroid = demean_points(test_pts_3d, demeaned_test_pts);

    cv::Mat covariance = cv::Mat(demeaned_train_pts).reshape(1, 0).t() * cv::Mat(demeaned_test_pts).reshape(1, 0);
    cv::SVD svd(covariance);
    //sgn of the determinant
    int s = sign(cv::determinant(svd.u * svd.vt));

    cv::Mat_<float> diagm = cv::Mat_<float>::eye(3, 3);
    //create a matrix with all ones but the lower right corner = S
    diagm(2, 2) = s;

    cv::Mat R = svd.u * diagm * svd.vt;
    R = R.t();
    cv::Mat T = cv::Mat(test_centroid).reshape(1, 0) - R * cv::Mat(train_centroid).reshape(1, 0);
    *R_out = R;
    *T_out = T;
    *matches_out = good_matches;
    *matches_mask = inlier_mask;
    float inlier_percentage = 100 * float(demeaned_train_pts.size()) / good_matches.size();
//     std::cout << "inlier percentage: " << inlier_percentage << std::endl;
//    std::cout << "number: " << good_matches.size() << std::endl;
    *found_out = inlier_percentage > *inlier_thresh && ((*min_inliers) < demeaned_test_pts.size());
    //std::cout << "pose good: " << (*found_out ? "YES" : "NO") << std::endl;
    return ecto::OK;
  }
  ecto::spore<cv::Mat> train_2d, test_2d, test_3d, train_3d, R_out, T_out;
  ecto::spore<matches_t> matches_in, matches_out;
  ecto::spore<cv::Mat> matches_mask;
  ecto::spore<bool> found_out;
  ecto::spore<unsigned> n_iters, min_inliers;
  ecto::spore<float> reprojection_error, inlier_thresh;

};
ECTO_CELL(features2d, MatchRefinement, "MatchRefinement",
          "A feature descriptor match refiner, using a Homography estimator");
ECTO_CELL(features2d, MatchRefinement3d, "MatchRefinement3d",
          "A feature descriptor match refiner, using an affine 3d to 3d estimator.");
ECTO_CELL(features2d, MatchRefinementPnP, "MatchRefinementPnP", "A feature descriptor match refiner, using PnP.");
ECTO_CELL(features2d, MatchRefinementHSvd, "MatchRefinementHSvd",
          "A feature descriptor match refiner, using Homography and svd estimation.");
