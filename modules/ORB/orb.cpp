#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <cmath>
#include "FASTHarris.h"

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

//disable show in here
#define DISABLE_SHOW 1
#if DISABLE_SHOW
#ifdef SHOW
#undef SHOW
#define SHOW() do{}while(false)
#endif
#endif

using ecto::tendrils;

struct Pyramid: ecto::module_interface
{

  void config(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    SHOW();
    levels_ = params.get<int> ("levels");
    scale_factor_ = params.get<float> ("scale_factor");
    magnification_ = params.get<int> ("magnification");
    for (int i = 0; i < levels_; i++)
    {
      outputs.declare<cv::Mat> (str(boost::format("out_%d") % i));
      outputs.declare<float> (str(boost::format("scale_%d") % i));
    }
    inputs.declare<cv::Mat> ("input");
  }

  void process(const tendrils& params, const tendrils& inputs, tendrils& outputs)
  {
    SHOW();
    const cv::Mat& in = inputs.get<cv::Mat> ("input");
    for (int i = 0; i < levels_; i++)
    {
      cv::Mat& out = outputs.get<cv::Mat> (str(boost::format("out_%d") % i));
      float & scale = outputs.get<float> (str(boost::format("scale_%d") % i));
      scale = 1.0f / float(std::pow(scale_factor_, i - magnification_));
      //use liner interp if magnifying, area based if decimating
      cv::resize(in, out, cv::Size(), scale, scale,
          i - magnification_ < 0 ? CV_INTER_LINEAR : CV_INTER_AREA);
    }
  }

  void initialize(tendrils& p)
  {
    SHOW();
    p.declare<int> ("levels", "Number of pyramid levels.", 3);
    p.declare<float> ("scale_factor", "The scale factor between levels", 1.42);
    p.declare<int> (
        "magnification",
        "The magnification, positive to start at a larger than real life. The scale at each pyramid level is 1.0/(scale_factor^{i - magnification}",
        0);
  }
  int levels_, magnification_;
  float scale_factor_;
};

struct PyramidRescale: ecto::module_interface
{
  void config(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    SHOW();
    levels_ = params.get<int> ("levels");
    for (int i = 0; i < levels_; i++)
    {
      inputs.declare<float> (str(boost::format("scale_%d") % i),
          "The scale of the level i");
      inputs.declare<std::vector<cv::KeyPoint> > (
          str(boost::format("kpts_%d") % i), "The kpts at level i.");
    }
    outputs.declare<std::vector<cv::KeyPoint> > ("out", "The rescaled kpts.");
  }
  void process(const tendrils& params, const tendrils& inputs, tendrils& outputs)
  {
    SHOW();
    std::vector<cv::KeyPoint>& kpts = outputs.get<std::vector<cv::KeyPoint> > (
        "out");
    kpts.clear();
    for (int i = 0; i < levels_; i++)
    {
      float scale = inputs.get<float> (str(boost::format("scale_%d") % i));
      const std::vector<cv::KeyPoint>& kpts_in = inputs.get<std::vector<
          cv::KeyPoint> > (str(boost::format("kpts_%d") % i));
      kpts.reserve(kpts.size() + kpts_in.size());
      for (size_t j = 0; j < kpts_in.size(); j++)
      {
        kpts.push_back(kpts_in[j]);
        cv::KeyPoint & x = kpts.back();
        x.octave = i;
        x.pt *= 1 / scale;
      }
    }

  }
  void initialize(tendrils& p)
  {
    SHOW();
    p.declare<int> ("levels", "Number of pyramid levels.", 3);
  }
  int levels_;
};
struct FAST: ecto::module_interface
{
  void config(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    SHOW();
    thresh_ = params.get<int> ("thresh");
    N_max_ = params.get<int> ("N_max");
    outputs.declare<std::vector<cv::KeyPoint> > ("out", "Detected keypoints");
    inputs.declare<cv::Mat> ("image", "The image to detect FAST on.");
    inputs.declare<cv::Mat> ("mask", "optional mask");
  }
  void process(const tendrils& params, const tendrils& inputs, tendrils& outputs)
  {
    SHOW();
    const cv::Mat& in = inputs.get<cv::Mat> ("image");
    const cv::Mat& mask = inputs.get<cv::Mat> ("mask");
    std::vector<cv::KeyPoint>& kpts = outputs.get<std::vector<cv::KeyPoint> > (
        "out");
    cv::FastFeatureDetector fd(thresh_, true);
    fd.detect(in, kpts, mask);
    if (int(kpts.size()) > N_max_)
    {
      std::nth_element(kpts.begin(), kpts.end() + N_max_, kpts.end(),
          FASTHarris::keypointResponseGreater);
      kpts.resize(N_max_);
    }
  }
  void initialize(tendrils& p)
  {
    SHOW();
    p.declare<int> ("thresh", "FAST threshhold.", 20);
    p.declare<int> ("N_max", "The maximum number of keypoints", 2000);
  }
  int thresh_, N_max_;
};

struct Harris: ecto::module_interface
{
  void config(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    SHOW();
    N_max_ = params.get<int> ("N_max");
    outputs.declare<std::vector<cv::KeyPoint> > ("out",
        "Detected keypoints, with Harris");
    inputs.declare<cv::Mat> ("image", "The image to calc harris response from.");
    inputs.declare<std::vector<cv::KeyPoint> > ("kpts",
        "The keypoints to fill with Harris response.");
  }
  void process(const tendrils& params, const tendrils& inputs, tendrils& outputs)
  {
    SHOW();
    const cv::Mat& image = inputs.get<cv::Mat> ("image");
    const std::vector<cv::KeyPoint>& kpts_in = inputs.get<std::vector<
        cv::KeyPoint> > ("kpts");
    std::vector<cv::KeyPoint>& kpts = outputs.get<std::vector<cv::KeyPoint> > (
        "out");
    FASTHarris::HarrisResponse h(image);
    kpts = kpts_in;
    h(kpts);
    if (int(kpts.size()) > N_max_)
    {
      std::nth_element(kpts.begin(), kpts.end() + N_max_, kpts.end(),
          FASTHarris::keypointResponseGreater);
      kpts.resize(N_max_);
    }
  }
  void initialize(tendrils& p)
  {
    SHOW();
    p.declare<int> ("N_max", "The maximum number of keypoints", 1000);
  }
  int N_max_;
};

struct DrawKeypoints: ecto::module_interface
{
  void config(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    SHOW();
    inputs.declare<cv::Mat> ("image", "The input image, to draw over.");
    outputs.declare<cv::Mat> ("image", "The output image.");
    inputs.declare<std::vector<cv::KeyPoint> > ("kpts",
        "The keypoints to draw.");
  }
  void process(const tendrils& params, const tendrils& inputs, tendrils& outputs)
  {
    SHOW();
    const cv::Mat& image = inputs.get<cv::Mat> ("image");
    const std::vector<cv::KeyPoint>& kpts_in = inputs.get<std::vector<
        cv::KeyPoint> > ("kpts");
    cv::Mat& out_image = outputs.get<cv::Mat> ("image");
    cv::drawKeypoints(image, kpts_in, out_image);
  }
  void initialize(tendrils& p)
  {
    SHOW();
  }
};

struct ScoreZipper: ecto::module_interface
{
  typedef std::vector<std::pair<float, float> > pairs_t;
  void config(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    SHOW();
    inputs.declare<std::vector<cv::KeyPoint> > ("kpts_0",
        "The keypoints to draw.");
    inputs.declare<std::vector<cv::KeyPoint> > ("kpts_1",
        "The keypoints to draw.");
    outputs.declare<pairs_t> ("scores", "the scores of the keypoint");
  }
  void process(const tendrils& params, const tendrils& inputs, tendrils& outputs)
  {
    SHOW();
    const std::vector<cv::KeyPoint>& kpts_in = inputs.get<std::vector<
        cv::KeyPoint> > ("kpts_0");
    const std::vector<cv::KeyPoint>& kpts_in2 = inputs.get<std::vector<
        cv::KeyPoint> > ("kpts_1");
    pairs_t& out = outputs.get<pairs_t> ("scores");
    out.clear();
    out.reserve(kpts_in.size());
    for (size_t i = 0, end = kpts_in.size(); i < end; ++i)
    {
      out.push_back(std::make_pair(kpts_in[i].response, kpts_in2[i].response));
      //std::cout << out.back().first << " " << out.back().second << std::endl;
    }
  }
  void initialize(tendrils& p)
  {
    SHOW();
  }
};

BOOST_PYTHON_MODULE(orb)
{
  namespace bp = boost::python;
  typedef std::vector<std::pair<float, float> > pairs_t;
  bp::class_<pairs_t> ("scores")
  .def(bp::vector_indexing_suite<pairs_t, false>() );
  bp::class_<std::pair<float,float> > ("pair_float")
  .def_readwrite("first",&std::pair<float,float>::first)
  .def_readwrite("second",&std::pair<float,float>::second);
  ecto::wrap<ScoreZipper>("ScoreZipper");
  ecto::wrap<Pyramid>("Pyramid");
  ecto::wrap<FAST>("FAST");
  ecto::wrap<Harris>("Harris");
  ecto::wrap<DrawKeypoints>("DrawKeypoints");
  ecto::wrap<PyramidRescale>("PyramidRescale");
}
