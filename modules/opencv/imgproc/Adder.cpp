#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using ecto::tendrils;
using ecto::spore;

namespace imgproc
{

  template<typename T>
  struct Adder
  {
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<T>("a", "to add to b");
      inputs.declare<T>("b", "to add to a");
      outputs.declare<T>("out", "a + b");
    }
    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      a_ = inputs["a"];
      b_ = inputs["b"];
      output_ = outputs["out"];

    }
    int
    process(const tendrils& /*inputs*/, const tendrils& /*outputs*/)
    {
      *output_ = T();
      *output_ = *a_ + *b_;
      return 0;
    }
    spore<T> a_, b_, output_;
  };
}

ECTO_CELL(imgproc, imgproc::Adder<cv::Mat>, "Adder", "Add an image.");
