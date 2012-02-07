#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "imgproc.h"
using ecto::tendrils;
using ecto::spore;
namespace imgproc
{
  struct MedianBlur
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare<int>("kernel", "kernel size, should be odd", 3);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      kernel_ = params["kernel"];
    }

    int
    process(const tendrils&, const tendrils&, const cv::Mat& input, cv::Mat& output)
    {
      cv::medianBlur(input, output, *kernel_);
      return ecto::OK;
    }
    spore<int> kernel_;
  };
}

using namespace imgproc;
ECTO_CELL(imgproc, Filter_<MedianBlur>, "MedianBlur", "Applies a median blur operator");
