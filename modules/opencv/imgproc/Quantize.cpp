#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "imgproc.h"

using ecto::tendrils;
namespace imgproc
{
  struct Quantize_
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare<double>("alpha", "Quantization factor", 10);
      p.declare<double>("beta", "Additive.", 10);

    }
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
    }
    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      alpha_ = params["alpha"];
      beta_ = params["beta"];
    }
    int
    process(const tendrils&, const tendrils&, const cv::Mat& input, cv::Mat& output)
    {
      double factor = *alpha_;
      double beta = *beta_;
      output = cv::Mat((input+beta) * (1. / factor)) * factor;
      return ecto::OK;
    }
    ecto::spore<double> alpha_, beta_;
  };

  //for pretty typeness.
  struct Quantize: Filter_<Quantize_>
  {
  };
}
ECTO_CELL(imgproc, imgproc::Quantize, "Quantize", "Divide and multiply an image by a factor.");
