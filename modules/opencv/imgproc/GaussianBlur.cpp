#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "imgproc.h"
using ecto::tendrils;
using ecto::spore;
namespace imgproc
{
  struct GaussianBlur
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare<int>("kernel", "kernel size, if zero computed from sigma", 0);
      p.declare<double>("sigma", "The first sigma in the guassian.", 1.0);

    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      kernel_ = params["kernel"];
      sigma_ = params["sigma"];
    }

    int
    process(const tendrils&, const tendrils&, const cv::Mat& input, cv::Mat& output)
    {
      cv::GaussianBlur(input, output, cv::Size(*kernel_,*kernel_), *sigma_);
      return ecto::OK;
    }
    spore<int> kernel_;
    spore<double> sigma_;
  };
}

using namespace imgproc;
ECTO_CELL(imgproc, Filter_<GaussianBlur>, "GaussianBlur", "Applies a gaussian blur operator");
