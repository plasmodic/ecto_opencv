#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv_candidate/hamming.h>
using ecto::tendrils;

typedef std::vector<cv::KeyPoint> kpts_t;
typedef std::vector<cv::DMatch> matches_t;
struct Matcher
{
  static void
  declare_params(tendrils& p)
  {
  }
  static void
  declare_io(const tendrils& p, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat>("train", "Test descriptors.");
    inputs.declare<cv::Mat>("test", "Train descriptors.");
    outputs.declare<matches_t>("matches", "The descriptor matches.");
  }
  int
  process(const tendrils& inputs, const tendrils& outputs)
  {
    cv::Mat train, test;
    inputs["train"] >> train;
    inputs["test"] >> test;
#if (CV_MAJOR_VERSION > 2) || ((CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION >= 4))
    cv::BFMatcher matcher(cv::NORM_HAMMING);
#else
    cv::BruteForceMatcher<HammingOperator> matcher;
#endif
    std::vector<cv::DMatch> matches;
    matcher.match(test, train, matches);
    outputs["matches"] << matches;
    return ecto::OK;
  }
};

ECTO_CELL(features2d, Matcher, "Matcher", "A feature descriptor matcher.");

