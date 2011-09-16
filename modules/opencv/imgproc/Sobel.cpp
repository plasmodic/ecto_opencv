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
      inputs.declare<cv::Mat>("image", "image.");
      outputs.declare<cv::Mat>("image", "sobel image");
    }
    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      x_ = params["x"];
      y_ = params["y"];
      input = inputs["image"];
      output = outputs["image"];
    }
    int
    process(const tendrils& /*inputs*/, const tendrils& /*outputs*/)
    {
      cv::Mat out;
      cv::Sobel(*input, out, CV_32F, *x_, *y_);
      *output = out;
      return ecto::OK;
    }
    ecto::spore<int> x_, y_;
    ecto::spore<cv::Mat> input, output;
  };
}
ECTO_CELL(imgproc, imgproc::Sobel, "Sobel", "Runs the sobel operator on the image.");
