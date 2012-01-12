#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "imgproc.h"
#include <iostream>

using ecto::tendrils;
namespace imgproc
{
  struct BitwiseNot
  {
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Mat>("input", "Image to not.").required(true);
      outputs.declare<cv::Mat>("out", "!input");
    }
    int
    process(const tendrils& in, const tendrils& out)
    {
      cv::Mat input, output;
      in["input"] >> input;
      cv::bitwise_not(input, output);
      out["out"] << output;
      return 0;
    }
  };
}
ECTO_CELL(imgproc, imgproc::BitwiseNot, "BitwiseNot", "Bitwise not the image.")
