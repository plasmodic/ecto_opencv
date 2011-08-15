#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "imgproc.h"
#include <iostream>

using ecto::tendrils;
namespace imgproc
{
  struct BitwiseAnd
  {
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Mat>("a", "to and with b");
      inputs.declare<cv::Mat>("b", "to and with a");
      outputs.declare<cv::Mat>("out", "a & b");
    }
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      cv::Mat a = inputs.get<cv::Mat>("a"), b = inputs.get<cv::Mat>("b");

      if (a.empty())
        throw std::runtime_error("a is empty");
      if (b.empty())
        throw std::runtime_error("b is empty");
      if (a.size() != b.size())
        throw std::runtime_error("a.size != b.size");
      cv::Mat output;
      cv::bitwise_and(a, b, output);
      outputs["out"] << output;
      return 0;
    }
  };
}
ECTO_CELL(imgproc, imgproc::BitwiseAnd, "BitwiseAnd", "Bitwise and the image.");
