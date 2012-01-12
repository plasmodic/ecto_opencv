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
    typedef Adder<T> C;
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&C::a_, "a", "to add to b");
      inputs.declare(&C::b_, "b", "to add to a");
      outputs.declare(&C::output_, "out", "a + b");
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
