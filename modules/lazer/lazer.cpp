#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <cmath>

using ecto::tendrils;

struct ScanLineDrawer
{
  static void declare_params(ecto::tendrils& params)
  {
    params.declare<float> ("scan_idx", "The scan line index, [0,1]", 0.5f);
    params.declare<bool> ("auto_scan",
        "After each process, increment the scanline", true);
  }
  static void declare_io(const tendrils& params, tendrils& inputs,
      tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("in", "The image to draw a scan line from.");
    outputs.declare<cv::Mat> ("out", "The scan line image.");

  }

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

  void configure(tendrils& params)
  {
    scan_idx_ = 0;//params.get<float> ("scan_idx");
    auto_scan_ = params.get<bool> ("auto_scan");
  }

  int process(const tendrils& inputs,
      tendrils& outputs)
  {
    const cv::Mat& in = inputs.get<cv::Mat> ("in");
    cv::Mat& out = outputs.get<cv::Mat> ("out");
    drawScaneLine(in, out, scan_idx_);
    if (auto_scan_)
      scan_idx_++;
    if (scan_idx_ >= in.rows)
      scan_idx_ = 0;
    return 0;
  }

  int scan_idx_;
  bool auto_scan_;
};

BOOST_PYTHON_MODULE(lazer)
{
  ecto::wrap<ScanLineDrawer>("ScanLineDrawer",
      "Draws a scanline in the image.\n"
        "Uses the intensity on the y axis, x position on the x axis.");
}
