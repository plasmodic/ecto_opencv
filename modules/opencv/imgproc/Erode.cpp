#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "imgproc.h"
#include <iostream>

using ecto::tendrils;
using ecto::spore;
namespace imgproc
{
  struct Erode_
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare<unsigned>("kernel", "Will determine the kernel size, kernl*2 + 1 is used so that the number is always odd.", 1);
      p.declare<Morph>("morph","Kernel shape", RECT);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      kernel_ = params["kernel"];
      morph_ = params["morph"];
    }

    int
    process(const tendrils&, const tendrils&, const cv::Mat& input, cv::Mat& output)
    {

      int kernel = *kernel_;
      cv::Size ksize(2*kernel+1,2*kernel+1); //only odd sizes.
      cv::Mat element = cv::getStructuringElement(*morph_, ksize);
      cv::erode(input,output, element);
      return ecto::OK;
    }
    spore<unsigned> kernel_;
    spore<Morph> morph_;
  };

  //for pretty typeness.
  struct Erode: Filter_<Erode_>
  {
  };
}

ECTO_CELL(imgproc, imgproc::Erode, "Erode", "Applies an erosion operator.");
