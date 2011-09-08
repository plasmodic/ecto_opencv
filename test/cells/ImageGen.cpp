#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using ecto::tendrils;
namespace opencv_test
{
  struct ImageGen
  {
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      outputs.declare<cv::Mat>("image", "A test image.");
    }
    int
    process(const tendrils& /*inputs*/, const tendrils& outputs)
    {
      cv::Mat out(cv::Size(640,480),CV_8UC3, cv::Scalar(125,255,255));
      outputs["image"] << out;
      return ecto::OK;
    }
  };
}
ECTO_CELL(opencv_test, opencv_test::ImageGen, "ImageGen", "Generate a test image.");
