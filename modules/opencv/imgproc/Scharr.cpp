#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using ecto::tendrils;
namespace imgproc
{
  struct Scharr
  {
    Scharr() { }

    static void declare_params(tendrils& p)
    {
      p.declare<int> ("x", "The derivative order in the x direction", 0);
      p.declare<int> ("y", "The derivative order in the y direction", 0);
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Mat> ("input", "image.");
      outputs.declare<cv::Mat> ("out", "scharr image");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      x_ = params["x"];
      y_ = params["y"];
    }
    int process(const tendrils& inputs, const tendrils& outputs)
    {
      cv::Scharr(inputs.get<cv::Mat> ("input"),
                 outputs.get<cv::Mat> ("out"), CV_32F, *x_, *y_);
      return 0;
    }
    ecto::spore<int> x_, y_;
  };
}
ECTO_CELL(imgproc, imgproc::Scharr, "Scharr", "Applies a schar operator");
