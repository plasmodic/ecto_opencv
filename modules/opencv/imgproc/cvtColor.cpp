#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "imgproc.h"
#include <iostream>

using ecto::tendrils;
namespace imgproc
{
  struct cvtColor_
  {
    static void
    declare_params(ecto::tendrils& p)
    {
      p.declare<Conversion>("flag", "Convertion type.", RGB2GRAY);
    }

    static void
    declare_io(const tendrils& p, tendrils& i, tendrils& o)
    {
    }

    void
    configure(const tendrils& p, const tendrils& i, const tendrils& o)
    {
      flag_ = p["flag"];
    }

    int
    process(const tendrils&, const tendrils&, const cv::Mat& input, cv::Mat& output)
    {
      int flag = *flag_;
      cv::cvtColor(input, output, flag);
      return ecto::OK;
    }
    ecto::spore<Conversion> flag_;
  };

  //for pretty typeness.
  struct cvtColor: Filter_<cvtColor_>
  {
  };
}

ECTO_CELL(imgproc, imgproc::cvtColor, "cvtColor", "Convert the color of a cv::Mat.");
