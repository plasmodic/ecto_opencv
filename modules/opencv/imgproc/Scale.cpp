#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include "imgproc.h"

using ecto::tendrils;
namespace imgproc
{
  struct Scale
  {
    static void
    declare_params(ecto::tendrils& p)
    {
      p.declare<float>("factor", "Scale the given image by the constant given", 1.0f);
      p.declare<Interpolation>("interpolation", "Interpolation method.", NN);
    }
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
    }
    void
    configure(const tendrils& p, const tendrils& inputs, const tendrils& outputs)
    {
      factor = p["factor"];
      interpolation = p["interpolation"];
    }
    int
    process(const tendrils&, const tendrils&, const cv::Mat& input, cv::Mat& output)
    {
      cv::Size nsize(input.size());
      nsize.width *= *factor;
      nsize.height *= *factor;
      cv::resize(input,output,nsize, *interpolation);
      return ecto::OK;
    }
    ecto::spore<float> factor;
    ecto::spore<Interpolation> interpolation;
  };

}
using namespace imgproc;
ECTO_CELL(imgproc,  Filter_<Scale>, "Scale", "Scales an image.");
