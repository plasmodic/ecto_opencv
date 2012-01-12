#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "imgproc.h"
#include <iostream>

using ecto::tendrils;
namespace imgproc
{
  struct cvtColor
  {
    static void
    declare_params(ecto::tendrils& p)
    {
      p.declare(&cvtColor::flag_, "flag", "Convertion type.", RGB2GRAY);
    }

    static void
    declare_io(const tendrils& p, tendrils& i, tendrils& o)
    {
    }

    void
    configure(const tendrils& p, const tendrils& i, const tendrils& o)
    {
    }

    int
    process(const tendrils&, const tendrils&, const cv::Mat& input, cv::Mat& output)
    {
      cv::cvtColor(input, output, *flag_);
      return ecto::OK;
    }
    ecto::spore<Conversion> flag_;
  };
}

using namespace imgproc;
ECTO_CELL(imgproc, Filter_<cvtColor>, "cvtColor", "Convert the color of a cv::Mat.");
