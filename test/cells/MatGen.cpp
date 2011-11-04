#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#define SHOW() std::cout << __PRETTY_FUNCTION__ << std::endl;
#define REFCOUNT(X)  std::cout << "ref count:" << ((X->refcount) ? *(X->refcount) : 0) << std::endl;

using ecto::tendrils;
namespace opencv_test
{
  struct MatGen
  {
    typedef MatGen C;
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      outputs.declare(&C::mat, "mat", "A test mat.");
    }
    int
    process(const tendrils& /*inputs*/, const tendrils& outputs)
    {
      cv::Mat out(std::rand() % 4 +1, std::rand() % 4 + 1, CV_32F);
      *mat = out;
      return ecto::OK;
    }
    ecto::spore<cv::Mat> mat;

  };
}
ECTO_CELL(opencv_test, opencv_test::MatGen, "MatGen", "Generate a test image.");
