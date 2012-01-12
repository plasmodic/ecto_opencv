#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core_c.h>
#include <iostream>
#include <vector>

using ecto::tendrils;
namespace imgproc
{
  struct SplitThree
  {
    static void
    declare_params(tendrils& p)
    {
      // p.declare<int>("n", "The number of the channel to select", 0);
    }
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Mat>("image", "image.");
      outputs.declare<cv::Mat>("image1", "First channel image");
      outputs.declare<cv::Mat>("image2", "Second channel image");
      outputs.declare<cv::Mat>("image3", "Third channel image");
    }
    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      input = inputs["image"];
      output1 = outputs["image1"];
      output2 = outputs["image2"];
      output3 = outputs["image3"];
    }
    int
    process(const tendrils& /*inputs*/, const tendrils& /*outputs*/)
    {
      std::vector<cv::Mat> three_channels;
      cv::split(*input, three_channels);
      *output1 = three_channels[0];
      *output2 = three_channels[1];
      *output3 = three_channels[2];

      return ecto::OK;
    }

    ecto::spore<cv::Mat> input, output1, output2, output3;
  };
}
ECTO_CELL(imgproc, imgproc::SplitThree, "SplitThree", "Turn an input image into three image planes of the color channels.");
