#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using ecto::tendrils;
namespace imgproc
{
  struct Canny
  {
    Canny() { }

    static void declare_params(tendrils& p)
    {
      p.declare<double> ("threshold1", "", 1.0);
      p.declare<double> ("threshold2", "", 1.0);
      p.declare<int> ("apertureSize", "", 3);
      p.declare<bool> ("L2gradient", "", false);
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Mat> ("input", "image.");
      outputs.declare<cv::Mat> ("out", "cannified image");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      thresh1 = params["threshold1"];
      thresh2 = params["threshold2"];
      apertureSize = params["apertureSize"];
      l2gradient = params["L2gradient"];
    }
    int process(const tendrils& inputs, const tendrils& outputs)
    {
      cv::Canny(inputs.get<cv::Mat> ("input"),
                outputs.get<cv::Mat> ("out"),
                *thresh1, *thresh2, *apertureSize, *l2gradient);

      return 0;
    }
    ecto::spore<double> thresh1, thresh2;
    ecto::spore<int> apertureSize;
    ecto::spore<bool> l2gradient;
  };
}
ECTO_CELL(imgproc, imgproc::Canny, "Canny", "Canny edge detection");
