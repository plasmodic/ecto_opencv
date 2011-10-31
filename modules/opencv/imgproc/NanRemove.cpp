#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "imgproc.h"
#include <iostream>
#include <numeric>

using ecto::tendrils;
namespace imgproc
{
  struct NanRemove_
  {
    static void
    declare_params(ecto::tendrils& p)
    {
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
      if(input.depth() != CV_32F) throw std::runtime_error("Expected input to be of floating point value, CV_32F");
      output.create(input.size(),input.type());
      cv::Mat_<float>::const_iterator begin(input.begin<float>()), end(input.end<float>());
      cv::Mat_<float>::iterator out_i(output.begin<float>());
      while(begin != end)
      {
        float v = *begin;
        if(std::isfinite(v))
        {
          *out_i = *begin;
        }
        else
        {
          *out_i = 0;
        }
        ++out_i;
        ++begin;
      }
      return ecto::OK;
    }
  };

  //for pretty typeness.
  struct NanRemove: Filter_<NanRemove_>
  {
  };
}

ECTO_CELL(imgproc, imgproc::NanRemove, "NanRemove", "Runs a nan removal filter on the image.");
