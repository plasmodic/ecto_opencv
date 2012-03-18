#pragma once

#include <opencv2/imgproc/imgproc.hpp>

namespace imgproc
{
  enum Interpolation
  {
    NN = CV_INTER_NN,
    LINEAR = CV_INTER_LINEAR,
    CUBIC = CV_INTER_CUBIC,
    AREA = CV_INTER_AREA,
    LANCZOS4 = CV_INTER_LANCZOS4,
  };

  enum Conversion
  {
    /* Constants for color conversion */
    BGR2BGRA = CV_BGR2BGRA,
    RGB2RGBA = CV_RGB2RGBA,

    BGRA2BGR = CV_BGRA2BGR,
    RGBA2RGB = CV_RGBA2RGB,

    BGR2RGBA = CV_BGR2RGBA,
    RGB2BGRA = CV_RGB2BGRA,

    RGBA2BGR = CV_RGBA2BGR,
    BGRA2RGB = CV_BGRA2RGB,

    BGR2RGB = CV_BGR2RGB,
    RGB2BGR = CV_RGB2BGR,

    BGRA2RGBA = CV_BGRA2RGBA,
    RGBA2BGRA = CV_RGBA2BGRA,

    BGR2GRAY = CV_BGR2GRAY,
    RGB2GRAY = CV_RGB2GRAY,
    GRAY2BGR = CV_GRAY2BGR,
    GRAY2RGB = CV_GRAY2RGB,
    GRAY2BGRA = CV_GRAY2BGRA,
    GRAY2RGBA = CV_GRAY2RGBA,
    BGRA2GRAY = CV_BGRA2GRAY,
    RGBA2GRAY = CV_RGBA2GRAY,

    BGR2BGR565 = CV_BGR2BGR565,
    RGB2BGR565 = CV_RGB2BGR565,
    BGR5652BGR = CV_BGR5652BGR,
    BGR5652RGB = CV_BGR5652RGB,
    BGRA2BGR565 = CV_BGRA2BGR565,
    RGBA2BGR565 = CV_RGBA2BGR565,
    BGR5652BGRA = CV_BGR5652BGRA,
    BGR5652RGBA = CV_BGR5652RGBA,

    GRAY2BGR565 = CV_GRAY2BGR565,
    BGR5652GRAY = CV_BGR5652GRAY,

    BGR2BGR555 = CV_BGR2BGR555,
    RGB2BGR555 = CV_RGB2BGR555,
    BGR5552BGR = CV_BGR5552BGR,
    BGR5552RGB = CV_BGR5552RGB,
    BGRA2BGR555 = CV_BGRA2BGR555,
    RGBA2BGR555 = CV_RGBA2BGR555,
    BGR5552BGRA = CV_BGR5552BGRA,
    BGR5552RGBA = CV_BGR5552RGBA,

    GRAY2BGR555 = CV_GRAY2BGR555,
    BGR5552GRAY = CV_BGR5552GRAY,

    BGR2XYZ = CV_BGR2XYZ,
    RGB2XYZ = CV_RGB2XYZ,
    XYZ2BGR = CV_XYZ2BGR,
    XYZ2RGB = CV_XYZ2RGB,

    BGR2YCrCb = CV_BGR2YCrCb,
    RGB2YCrCb = CV_RGB2YCrCb,
    YCrCb2BGR = CV_YCrCb2BGR,
    YCrCb2RGB = CV_YCrCb2RGB,

    BGR2HSV = CV_BGR2HSV,
    RGB2HSV = CV_RGB2HSV,

    BGR2Lab = CV_BGR2Lab,
    RGB2Lab = CV_RGB2Lab,

    BayerBG2BGR = CV_BayerBG2BGR,
    BayerGB2BGR = CV_BayerGB2BGR,
    BayerRG2BGR = CV_BayerRG2BGR,
    BayerGR2BGR = CV_BayerGR2BGR,

    BayerBG2RGB = CV_BayerBG2RGB,
    BayerGB2RGB = CV_BayerGB2RGB,
    BayerRG2RGB = CV_BayerRG2RGB,
    BayerGR2RGB = CV_BayerGR2RGB,

    BGR2Luv = CV_BGR2Luv,
    RGB2Luv = CV_RGB2Luv,
    BGR2HLS = CV_BGR2HLS,
    RGB2HLS = CV_RGB2HLS,

    HSV2BGR = CV_HSV2BGR,
    HSV2RGB = CV_HSV2RGB,

    Lab2BGR = CV_Lab2BGR,
    Lab2RGB = CV_Lab2RGB,
    Luv2BGR = CV_Luv2BGR,
    Luv2RGB = CV_Luv2RGB,
    HLS2BGR = CV_HLS2BGR,
    HLS2RGB = CV_HLS2RGB,

    BayerBG2BGR_VNG = CV_BayerBG2BGR_VNG,
    BayerGB2BGR_VNG = CV_BayerGB2BGR_VNG,
    BayerRG2BGR_VNG = CV_BayerRG2BGR_VNG,
    BayerGR2BGR_VNG = CV_BayerGR2BGR_VNG,

    BayerBG2RGB_VNG = CV_BayerBG2RGB_VNG,
    BayerGB2RGB_VNG = CV_BayerGB2RGB_VNG,
    BayerRG2RGB_VNG = CV_BayerRG2RGB_VNG,
    BayerGR2RGB_VNG = CV_BayerGR2RGB_VNG,

    BGR2HSV_FULL = CV_BGR2HSV_FULL,
    RGB2HSV_FULL = CV_RGB2HSV_FULL,
    BGR2HLS_FULL = CV_BGR2HLS_FULL,
    RGB2HLS_FULL = CV_RGB2HLS_FULL,

    HSV2BGR_FULL = CV_HSV2BGR_FULL,
    HSV2RGB_FULL = CV_HSV2RGB_FULL,
    HLS2BGR_FULL = CV_HLS2BGR_FULL,
    HLS2RGB_FULL = CV_HLS2RGB_FULL,

    LBGR2Lab = CV_LBGR2Lab,
    LRGB2Lab = CV_LRGB2Lab,
    LBGR2Luv = CV_LBGR2Luv,
    LRGB2Luv = CV_LRGB2Luv,

    Lab2LBGR = CV_Lab2LBGR,
    Lab2LRGB = CV_Lab2LRGB,
    Luv2LBGR = CV_Luv2LBGR,
    Luv2LRGB = CV_Luv2LRGB,

    BGR2YUV = CV_BGR2YUV,
    RGB2YUV = CV_RGB2YUV,
    YUV2BGR = CV_YUV2BGR,
    YUV2RGB = CV_YUV2RGB,

    BayerBG2GRAY = CV_BayerBG2GRAY,
    BayerGB2GRAY = CV_BayerGB2GRAY,
    BayerRG2GRAY = CV_BayerRG2GRAY,
    BayerGR2GRAY = CV_BayerGR2GRAY,
  };

  enum Morph
  {
    RECT = cv::MORPH_RECT, CROSS = cv::MORPH_CROSS, ELLIPSE = cv::MORPH_ELLIPSE
  };

  template<typename T>
  struct Filter_ : T
  {
    T* thiz()
    {
      return static_cast<T*>(this);
    }

    static void
    declare_params(ecto::tendrils& p)
    {
      T::declare_params(p);
    }

    static void
    declare_io(const ecto::tendrils& p, ecto::tendrils& i, ecto::tendrils& o)
    {
      i.declare(&Filter_::input_,"image", "An image.").required(true);
      o.declare(&Filter_::output_,"image", "The filtered image.");
    }

    void
    configure(const ecto::tendrils& p, const ecto::tendrils& i, const ecto::tendrils& o)
    {
      thiz()->configure(p, i, o);
    }

    int
    process(const ecto::tendrils& i, const ecto::tendrils& o)
    {
      *output_ = cv::Mat(); //reset the output so that the cv mat is reallocated
      if (input_->empty())
        return ecto::OK;
      return thiz()->process(i, o,*input_, *output_);
    }
    ecto::spore<cv::Mat> input_, output_;
  };
}
