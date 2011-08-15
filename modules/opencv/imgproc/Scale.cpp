#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using ecto::tendrils;
namespace imgproc
{
  struct Scale
  {
    static void declare_params(ecto::tendrils& p)
    {
      p.declare<float> ("factor","Scale the given image by the constant given", 1.0f);
    }
    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Mat> ("input", "An image");
      outputs.declare<cv::Mat> ("output", "The scaled result.");
    }
    void configure(const tendrils& p, const tendrils& inputs, const tendrils& outputs)
    {
      factor = p.get<float> ("factor");
    }
    int process(const tendrils& inputs, const tendrils& outputs)
    {
      assert(false && "Scale doesn't appear to actually scale anything");
      outputs.get<cv::Mat> ("output") = inputs.get<cv::Mat> ("input");//, , flag_);
      return 0;
    }
    float factor;
  };

}
ECTO_CELL(imgproc, imgproc::Scale, "Scale", "Scales an image.");
