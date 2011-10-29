#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using ecto::tendrils;
namespace opencv_test
{
  struct ImageGen
  {
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      outputs.declare<cv::Mat>("image", "A test image.");
    }
    int
    process(const tendrils& /*inputs*/, const tendrils& outputs)
    {
      cv::Mat out(cv::Size(640,480),CV_8UC3, cv::Scalar(cv::rand()%255,cv::rand()%255,cv::rand()%255));
      outputs["image"] << out;
      return ecto::OK;
    }
  };


  struct ImageDelay
  {
    typedef ImageDelay C;

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&C::in,"image", "A test image.");
      outputs.declare(&C::out,"image", "A test image.");
    }

    int
    process(const tendrils& /*inputs*/, const tendrils& outputs)
    {
      boost::this_thread::sleep(boost::posix_time::milliseconds(cv::rand()%1000));
      cv::Mat outm;
      in->copyTo(outm);
      boost::this_thread::sleep(boost::posix_time::milliseconds(cv::rand()%1000));
      out << outm;
      return ecto::OK;
    }
    ecto::spore<cv::Mat> in,out;
  };
  struct ImageCmp
  {
    typedef ImageCmp C;

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&C::in1,"image1", "A test image.");
      inputs.declare(&C::in2,"image2", "A test image.");
    }

    int
    process(const tendrils& /*inputs*/, const tendrils& outputs)
    {
      cv::Mat cmp = in1 - in2;
      if( cv::countNonZero(cmp) != 0)
      {
        throw std::runtime_error("WTF images should be the same");
      }
      return ecto::OK;
    }
    ecto::spore<cv::Mat> in1,in2;
  };
}
ECTO_CELL(opencv_test, opencv_test::ImageGen, "ImageGen", "Generate a test image.");
ECTO_CELL(opencv_test, opencv_test::ImageCmp, "ImageCmp", "Generate a test image.");
ECTO_CELL(opencv_test, opencv_test::ImageDelay, "ImageDelay", "Generate a test image.");
