#include <ecto/ecto.hpp>
#include "imgproc.h"

using ecto::tendrils;

//struct ChannelSplitter
//{
//  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
//  {
//    inputs.declare<cv::Mat> ("input", "The 3 channel image to split.");
//    outputs.declare<cv::Mat> ("out_0", "Channel 0.");
//    outputs.declare<cv::Mat> ("out_1", "Channel 1.");
//    outputs.declare<cv::Mat> ("out_2", "Channel 2.");
//  }
//  int process(const tendrils& inputs, const tendrils& outputs)
//  {
//    const cv::Mat& in = inputs.get<cv::Mat> ("input");
//    if (in.channels() == 3)
//      cv::split(in, channels_);
//    else if (in.channels() == 1)
//    {
//      for (int i = 0; i < 3; i++)
//      {
//        channels_[i] = in;
//      }
//    }
//    else
//    {
//      throw std::runtime_error("unsupported number of channels! must be 1 or 3");
//    }
//    outputs.get<cv::Mat> ("out_0") = channels_[0];
//    outputs.get<cv::Mat> ("out_1") = channels_[1];
//    outputs.get<cv::Mat> ("out_2") = channels_[2];
//    return 0;
//  }
//  cv::Mat channels_[3];
//};
//ECTO_CELL(imgproc, ChannelSplitter, "ChannelSplitter", "Splits an image into 3 channels.");
//
//
//
//struct AbsNormalized
//{
//  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
//  {
//    inputs.declare<cv::Mat> ("input", "image.");
//    outputs.declare<cv::Mat> ("out", "absolute and normalized");
//  }
//  int process(const tendrils& inputs, const tendrils& outputs)
//  {
//    cv::Mat m = inputs.get<cv::Mat> ("input");
//    //grab a reference
//    cv::Mat & out = outputs.get<cv::Mat> ("out");
//    out = cv::abs(m) / (cv::norm(m, cv::NORM_INF) * 0.5);
//    return 0;
//  }
//};
//ECTO_CELL(imgproc, AbsNormalized, "AbsNormalized", "Takes the absolute value of the image and normalizes")

