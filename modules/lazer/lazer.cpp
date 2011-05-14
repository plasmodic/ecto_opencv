#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <cmath>

//disable show in here
#define DISABLE_SHOW 1
#if DISABLE_SHOW
#ifdef SHOW
#undef SHOW
#define SHOW() do{}while(false)
#endif
#endif

struct ScanLineDrawer : ecto::module
{
  static void drawScaneLine(const cv::Mat& image, cv::Mat& out, int line_n)
  {
    out.create(260, image.cols, CV_8UC1);
    out = cv::Scalar(0);
    if (line_n >= image.rows || line_n < 0)
      throw std::logic_error("badness");

    for (int i = 0; i < image.cols; i++)
    {
      uchar val = image.at<uchar> (line_n, i);
      out.at<uchar> (out.rows - val, i) = 255;
    }
  }
  void Config()
  {
    SHOW();
    inputs.declare<cv::Mat> ("in", "The image to draw a scan line from.");
    outputs.declare<cv::Mat> ("out", "The scan line image.");
    scan_idx_ = 0;//params.get<float> ("scan_idx");
    auto_scan_ = params.get<bool> ("auto_scan");
  }
  void Process()
  {
    SHOW();
    const cv::Mat& in = inputs.get<cv::Mat> ("in");
    cv::Mat& out = outputs.get<cv::Mat> ("out");
    drawScaneLine(in, out, scan_idx_);
    if (auto_scan_)
      scan_idx_++;
    if (scan_idx_ >= in.rows)
      scan_idx_ = 0;
  }
  static void Initialize(ecto::tendrils& params)
  {
    SHOW();
    params.declare<float> ("scan_idx", "The scan line index, [0,1]", 0.5f);
    params.declare<bool> ("auto_scan", "After each process, increment the scanline", true);
  }
  int scan_idx_;
  bool auto_scan_;
};

struct mm : ecto::module
{
  void Config()
  {
    SHOW();
    inputs.declare<cv::Mat> ("in", "The image to to find a vertical lazer line in.");
    outputs.declare<cv::Mat> ("out", "The lazer image (0 - no lazer, 255 - lazer).");
  }
  void Process()
  {
    SHOW();
    //const cv::Mat& in = inputs.get<cv::Mat> ("in");
    //cv::Mat& out = outputs.get<cv::Mat> ("out");
  }
  static void Initialize(ecto::tendrils& params)
  {
    SHOW();
  }
};

struct LaserDetector : ecto::module
{
  void Config()
  {
    SHOW();
    inputs.declare<cv::Mat> ("in", "The image to to find a vertical lazer line in.");
    outputs.declare<cv::Mat> ("out", "The lazer image (0 - no lazer, 255 - lazer).");
  }
  void Process()
  {
    SHOW();
    //const cv::Mat& in = inputs.get<cv::Mat> ("in");
    //cv::Mat& out = outputs.get<cv::Mat> ("out");
  }
  static void Initialize(ecto::tendrils& params)
  {
    SHOW();
  }
};

BOOST_PYTHON_MODULE(lazer)
{
  ecto::wrap<ScanLineDrawer>("ScanLineDrawer");
  ecto::wrap<LaserDetector>("LaserDetector");
}
