#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core_c.h>
#include <iostream>
#include <vector>

using ecto::tendrils;
namespace imgproc
{
  struct Split
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare<int>("n", "The number of the channel to select", 0);
    }
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Mat>("image", "image.");
      outputs.declare<cv::Mat>("image", "channel image");
    }
    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      n_ = params["n"];
      input = inputs["image"];
      output = outputs["image"];
    }
    int
    process(const tendrils& /*inputs*/, const tendrils& /*outputs*/)
    {
      std::vector<cv::Mat> three_channels;
      cv::split(*input, three_channels);
      *output = three_channels[*n_];

      return ecto::OK;
    }
    ecto::spore<int> n_;
    ecto::spore<cv::Mat> input, output;
  };
}
ECTO_CELL(imgproc, imgproc::Split, "Split", "Returns the n'th color channel from the image.");
