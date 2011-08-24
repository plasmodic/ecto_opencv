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

      if (a.empty() && b.empty())
        throw std::runtime_error("a and b are empty");
      if (a.empty())
      {
        outputs["out"] << b;
      }
      else if (b.empty())
      {
        outputs["out"] << a;
      }else
      {
        cv::Mat output;
        cv::bitwise_and(a, b, output);
        outputs["out"] << output;
      }
      return ecto::OK;
    }
  };
}
ECTO_CELL(imgproc, imgproc::BitwiseAnd, "BitwiseAnd", "Bitwise and the image.");
