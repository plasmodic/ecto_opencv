#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "imgproc.h"
#include <iostream>

using ecto::tendrils;
using ecto::spore;
namespace imgproc
{
  struct GaussianBlur
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare<int>("kernel", "kernel size, if zero computed from sigma", 0);
      p.declare<double>("sigma", "The first sigma in the guassian.", 1.0);

    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Mat>("input", "image.");
      outputs.declare<cv::Mat>("out", "blurred image");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      kernel_ = params["kernel"];
      sigma_ = params["sigma"];
      input_ = inputs["input"];
      output_ = outputs["out"];
    }
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      *output_ = cv::Mat();
      cv::GaussianBlur(*input_, *output_, cv::Size(*kernel_,*kernel_), *sigma_);
      return 0;
    }
    spore<cv::Mat> input_,output_;
    spore<int> kernel_;
    spore<double> sigma_;
  };
}

ECTO_CELL(imgproc, imgproc::GaussianBlur, "GaussianBlur", "Applies a gaussian blur operator");
