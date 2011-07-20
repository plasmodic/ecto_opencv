#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using ecto::tendrils;
namespace imgproc
{
  struct Sobel
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare<int>("x", "The derivative order in the x direction", 0);
      p.declare<int>("y", "The derivative order in the y direction", 0);
    }
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Mat>("input", "image.");
      outputs.declare<cv::Mat>("out", "sobel image");
    }
    void
    configure(tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      x_ = params.get<int>("x");
      y_ = params.get<int>("y");
      input = inputs.at("input");
      output = outputs.at("output");
    }
    int
    process(const tendrils& inputs, tendrils& outputs)
    {
      cv::Mat out;
      cv::Sobel(input.read(), *output, CV_32F, x_, y_);
      return ecto::OK;
    }
    int x_, y_;
    ecto::spore<cv::Mat> input, output;
  };
}
ECTO_CELL(imgproc, imgproc::Sobel, "Sobel", "Runs the sobel operator on the image.");
