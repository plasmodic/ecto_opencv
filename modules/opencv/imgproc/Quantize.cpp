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
      output = cv::Mat((input + beta) * (1. / factor)) * factor;
      return ecto::OK;
    }
    ecto::spore<double> alpha_, beta_;
  };

  struct ConvertTo_
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare(&ConvertTo_::alpha_, "alpha", "Factor", 1);
      p.declare(&ConvertTo_::beta_, "beta", "Additive.", 0);
      p.declare(&ConvertTo_::type_, "cv_type", "The cv type for conversion, if -1 use the source type.", -1);

    }
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
    }
    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
    }
    int
    process(const tendrils&, const tendrils&, const cv::Mat& input, cv::Mat& output)
    {
      double factor = *alpha_;
      double beta = *beta_;
      int type = input.type();
      if (*type_ != -1)
      {
        type = *type_;
      }
      input.convertTo(output, type, factor, beta);
      return ecto::OK;
    }
    ecto::spore<double> alpha_, beta_;
    ecto::spore<int> type_;
  };
  //for pretty typeness.
  struct Quantize: Filter_<Quantize_>
  {
  };

  //for pretty typeness.
  struct ConvertTo: Filter_<ConvertTo_>
  {
  };
}
ECTO_CELL(imgproc, imgproc::Quantize, "Quantize", "Divide and multiply an image by a factor.");
ECTO_CELL(imgproc, imgproc::ConvertTo, "ConvertTo", "Convert to.");
