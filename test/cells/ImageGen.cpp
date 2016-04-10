#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#define SHOW() std::cout << __PRETTY_FUNCTION__ << std::endl;
#if CV_MAJOR_VERSION == 3
#define REFCOUNT(X)  std::cout << "ref count:" << ((X->u) ? (X->u->refcount) : 0) << std::endl;
#else
#define REFCOUNT(X)  std::cout << "ref count:" << ((X->refcount) ? *(X->refcount) : 0) << std::endl;
#endif

using ecto::tendrils;
namespace opencv_test
{
  struct ImageGen
  {
    typedef ImageGen C;
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      outputs.declare(&C::image, "image", "A test image.");
    }
    int
    process(const tendrils& /*inputs*/, const tendrils& outputs)
    {
      SHOW()

//      *image = cv::Mat();
      REFCOUNT(image);
      cv::Mat out(cv::Size(640, 480), CV_8UC3, cv::Scalar(std::rand() % 255, std::rand() % 255, std::rand() % 255));
      out.copyTo(*image);
      return ecto::OK;
    }
    ecto::spore<cv::Mat> image;
  };

  struct ImageDelay
  {
    typedef ImageDelay C;

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&C::in, "image", "A test image.");
      outputs.declare(&C::out, "image", "A test image.");
    }

    int
    process(const tendrils& /*inputs*/, const tendrils& outputs)
    {
      SHOW()
      boost::this_thread::sleep(boost::posix_time::milliseconds(std::rand() % 1000));
      REFCOUNT(in);
      REFCOUNT(out);
      in->copyTo(*out);
      return ecto::OK;
    }
    ecto::spore<cv::Mat> in, out;
  };
  struct ImageCmp
  {
    typedef ImageCmp C;

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&C::in1, "image1");
      inputs.declare(&C::in2, "image2");
      outputs.declare(&C::out, "out");
    }

    int
    process(const tendrils& /*inputs*/, const tendrils& outputs)
    {
      cv::Mat diffm = *in1 - *in2;
      *out = diffm;
      std::cout << diffm.rows << std::endl;
      if (cv::countNonZero(diffm.reshape(1, 1)) != 0)
      {
        throw std::runtime_error("WTF images should be the same");
      }
      return ecto::OK;
    }
    ecto::spore<cv::Mat> in1, in2, out;
  };
}
ECTO_CELL(opencv_test, opencv_test::ImageGen, "ImageGen", "Generate a test image.");
ECTO_CELL(opencv_test, opencv_test::ImageCmp, "ImageCmp", "Generate a test image.");
ECTO_CELL(opencv_test, opencv_test::ImageDelay, "ImageDelay", "Generate a test image.");
