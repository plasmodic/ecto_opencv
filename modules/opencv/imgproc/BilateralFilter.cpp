#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "imgproc.h"
#include <iostream>

using ecto::tendrils;
namespace imgproc
{
  struct bilateral_
  {
    static void
    declare_params(ecto::tendrils& p)
    {
      p.declare<int>(
          "d",
          "Diameter of each pixel neighborhood that is used during filtering. If it is non-positive, it is computed from sigmaSpace .",
          -1);
      p.declare<double>(
          "sigmaColor",
          "Filter sigma in the color space. A larger value of the parameter means that farther colors within the pixel "
          "neighborhood (see sigmaSpace ) will be mixed together, resulting in larger areas of semi-equal color.",
          25);
      p.declare<double>(
          "sigmaSpace",
          "Filter sigma in the coordinate space. A larger value of the parameter means that farther pixels will "
          "influence each other as long as their colors are close enough (see sigmaColor ). When d>0 , it specifies "
          "the neighborhood size regardless of sigmaSpace . Otherwise, d is proportional to sigmaSpace .",
          3);
    }

    static void
    declare_io(const tendrils& p, tendrils& i, tendrils& o)
    {
    }

    void
    configure(const tendrils& p, const tendrils& i, const tendrils& o)
    {
      diameter_ = p["d"];
      sigmaColor_ = p["sigmaColor"];
      sigmaSpace_ = p["sigmaSpace"];
    }

    int
    process(const tendrils&, const tendrils&, const cv::Mat& input, cv::Mat& output)
    {
      cv::bilateralFilter(input, output, *diameter_, *sigmaColor_, *sigmaSpace_);
      return ecto::OK;
    }
    ecto::spore<int> diameter_;
    ecto::spore<double> sigmaColor_, sigmaSpace_;
  };

  //for pretty typeness.
  struct BilateralFilter: Filter_<bilateral_>
  {
  };
}

ECTO_CELL(imgproc, imgproc::BilateralFilter, "BilateralFilter", "Runs a bilateral filter on the image.");
