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
    onvalue_change_x(int x)
    {
      x_ = x;
    }
    void
    onvalue_change_y(int y) 
    {
      y_ = y;
    }
    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      params["x"] >> x_;
      params["y"] >> y_;
      input = inputs["input"];
      output = outputs["out"];
      params["x"]->set_callback<int>(boost::bind(&Sobel::onvalue_change_x, this, _1));
      params["y"]->set_callback<int>(boost::bind(&Sobel::onvalue_change_y, this, _1));
    }
    int
    process(const tendrils& /*inputs*/, const tendrils& /*outputs*/)
    {
      cv::Mat out;
      cv::Sobel(*input, out, CV_32F, x_, y_);
      *output = out;
      return ecto::OK;
    }
    int x_, y_;
    ecto::spore<cv::Mat> input, output;
  };
}
ECTO_CELL(imgproc, imgproc::Sobel, "Sobel", "Runs the sobel operator on the image.");
