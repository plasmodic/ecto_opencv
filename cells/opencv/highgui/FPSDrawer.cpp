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
  struct FPSDrawer
  {
    static void
    draw(cv::Mat& drawImage, float freq, double scale)
    {
      using namespace cv;
      Size imgsize = drawImage.size();
      std::string scaleText = boost::str(boost::format("%ux%u @ %.2f Hz") % imgsize.width % imgsize.height % freq);
      int baseline = 0;
      Size sz = getTextSize(scaleText, CV_FONT_HERSHEY_SIMPLEX, scale, 1, &baseline);
      rectangle(drawImage, Point(10, 30 + 5), Point(10, 30) + Point(sz.width, -sz.height - 5), Scalar::all(0), -1);
      putText(drawImage, scaleText, Point(10, 30), CV_FONT_HERSHEY_SIMPLEX, scale, Scalar::all(255), 1, CV_AA, false);
    }

    static void
    declare_params(tendrils& params)
    {
      params.declare(&FPSDrawer::scale_, "scale", "Sets FPS font scale size", 1.0);
    }

    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare<cv::Mat>("image", "The original image to draw the pose onto.");
      out.declare<cv::Mat>("image", "The image with fps drawn on it.");
    }

    FPSDrawer()
      : count(), freq()
    { }
    
    int
    process(const tendrils& in, const tendrils& out)
    {
      pt::ptime now = pt::microsec_clock::universal_time();
      if (count == 0)
      {
        prev = now;
      }
      else if (count == 30)
      {
        pt::time_duration elapsed = now - prev;
        freq = double(count) / (elapsed.total_microseconds() * 1e-6);
        prev = now;
        count = 0;
      }
      count++;
      cv::Mat image, output;
      in["image"] >> image;
      image.copyTo(output);
      draw(output, freq, *scale_);
      out["image"] << output;
      return 0;
    }
    pt::ptime prev;
    ecto::spore<double> scale_;
    size_t count;
    double freq;
  };
}
ECTO_CELL(highgui, ecto_opencv::FPSDrawer, "FPSDrawer", "Draw the Hz on an image.");
