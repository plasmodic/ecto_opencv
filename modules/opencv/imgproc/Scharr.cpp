#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "imgproc.h"

using ecto::tendrils;
namespace imgproc
{
  struct Scharr
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare(&Scharr::x_, "x", "The derivative order in the x direction", 0);
      p.declare(&Scharr::y_, "y", "The derivative order in the y direction", 0);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
    }
    int
    process(const tendrils& inputs, const tendrils& outputs, const cv::Mat& input, cv::Mat& output)
    {
      cv::Scharr(input, output, CV_32F, *x_, *y_);
      return ecto::OK;
    }
    ecto::spore<int> x_, y_;
  };
}
using namespace imgproc;
ECTO_CELL(imgproc, Filter_<Scharr>, "Scharr", "Applies a scharr operator to the input image.");
