#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using ecto::tendrils;
namespace imgproc
{
  struct cvtColor
  {
    static void
    declare_params(ecto::tendrils& p)
    {
      std::stringstream ss;
      ss << "Convert an image's color using opencv, possible flags are:\n" << " RGB2GRAY = " << CV_RGB2GRAY << "\n"
         << " RGB2BGR = " << CV_RGB2BGR << "\n" << " RGB2LAB = " << CV_RGB2Lab << "\n" << " BGR2LAB = " << CV_BGR2Lab
         << "\n" << " GRAY2RGB = " << CV_GRAY2RGB;
      p.declare<int>("flag", ss.str(), CV_RGB2BGR);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Mat>("input", "Color image.");
      outputs.declare<cv::Mat>("out", "input as a Gray image.");
    }

    void
    configure(tendrils& p, tendrils& inputs, tendrils& outputs)
    {
      flag_ = p.get<int>("flag");
    }

    int
    process(const tendrils& inputs, tendrils& outputs)
    {
      cv::cvtColor(inputs.get<cv::Mat>("input"), outputs.get<cv::Mat>("out"), flag_);
      return 0;
    }

    int flag_;
  };
}

ECTO_CELL(imgproc, imgproc::cvtColor, "cvtColor", "Convert the color of a cv::Mat");
