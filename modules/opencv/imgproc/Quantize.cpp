#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using ecto::tendrils;
namespace imgproc
{
  struct Quantize
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare<double>("factor", "Quantization factor", 10);
    }
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Mat>("image", "Input image.");
      outputs.declare<cv::Mat>("image", "Quantized output image.");
    }
    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      factor_ = params["factor"];
      input = inputs["image"];
      output = outputs["image"];
    }
    int
    process(const tendrils& /*inputs*/, const tendrils& /*outputs*/)
    {
      double factor = *factor_;
      cv::Mat out = cv::Mat((*input) * (1. / factor)) * factor;
      *output = out;
      return ecto::OK;
    }
    ecto::spore<double> factor_;
    ecto::spore<cv::Mat> input, output;
  };
}
ECTO_CELL(imgproc, imgproc::Quantize, "Quantize", "Divide and multiply an image by a factor.");
