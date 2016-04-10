#include <ecto/ecto.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <string>
using ecto::tendrils;

namespace pt = boost::posix_time;
namespace fs = boost::filesystem;
namespace ecto_opencv
{
  using cv::Point;
  using cv::Scalar;

  struct DoubleDrawer
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare(&DoubleDrawer::fmt, "format", "boost::format string.  Will be passed a double",
                std::string("%f"));
      p.declare(&DoubleDrawer::x, "x", "x offset", 0);
      p.declare(&DoubleDrawer::y, "y", "y offset", 0);
    }

    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare(&DoubleDrawer::value, "value", "Value to be drawn");
      in.declare(&DoubleDrawer::inimg, "image", "The original image to draw the pose onto.");
      out.declare(&DoubleDrawer::outimg, "image", "The image with fps drawn on it.");
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      cv::Mat image, output;
      inimg->copyTo(*outimg);

      std::string fmtedtxt = boost::str(boost::format(*fmt) % *value);
      int baseline = 0;
      cv::Size sz = cv::getTextSize(fmtedtxt, CV_FONT_HERSHEY_SIMPLEX, 1, 1, &baseline);
      cv::rectangle(*outimg,
                    Point(10 + *x, 30 + *y + 5),
                    Point(10 + *x, 30 + *y) + Point(sz.width, -sz.height - 5),
                    Scalar::all(0), -1);
      cv::putText(*outimg, fmtedtxt, Point(10 + *x, 30 + *y), CV_FONT_HERSHEY_SIMPLEX,
                  1.0, Scalar::all(255), 1, CV_AA, false);

      return 0;
    }
    pt::ptime prev;
    size_t count;
    double freq;
    ecto::spore<std::string> fmt;
    ecto::spore<unsigned> x, y;
    ecto::spore<double> value;
    ecto::spore<cv::Mat> inimg, outimg;
  };
}
ECTO_CELL(highgui, ecto_opencv::DoubleDrawer, "DoubleDrawer", "Draw a double and legend on an image.");
