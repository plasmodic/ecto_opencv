#include <ecto/ecto.hpp>

#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>

//disable show in here
#if 1
#ifdef SHOW
#undef SHOW
#define SHOW() do{}while(false)
#endif
#endif

using ecto::tendrils;

struct cvtColor: ecto::module_interface
{
  void configure(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    SHOW();
    flag_ = params.get<int> ("flag");
    inputs.declare<cv::Mat> ("input", "Color image.");
    outputs.declare<cv::Mat> ("out", "input as a Gray image.");
  }
  void process(const tendrils& params, const tendrils& inputs,
      tendrils& outputs)
  {
    SHOW();
    cv::cvtColor(inputs.get<cv::Mat> ("input"), outputs.get<cv::Mat> ("out"),
        flag_);
  }
  void initialize(ecto::tendrils& p)
  {
    SHOW();
    std::stringstream ss;
    ss << "Convert an image's color using opencv, possible flags are:\n"
        << " RGB2GRAY = " << CV_RGB2GRAY << "\n" << " RGB2BGR = " << CV_RGB2BGR
        << "\n" << " RGB2LAB = " << CV_RGB2Lab << "\n" << " BGR2LAB = "
        << CV_BGR2Lab;
    p.declare<int> ("flag", ss.str(), CV_RGB2BGR);
  }
  int flag_;
};

struct ChannelSplitter: ecto::module_interface
{
  void configure(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    SHOW();
    inputs.declare<cv::Mat> ("input", "The 3 channel image to split.");
    outputs.declare<cv::Mat> ("out_0", "Channel 0.");
    outputs.declare<cv::Mat> ("out_1", "Channel 1.");
    outputs.declare<cv::Mat> ("out_2", "Channel 2.");
  }
  void process(const tendrils& params, const tendrils& inputs,
      tendrils& outputs)
  {
    SHOW();
    const cv::Mat& in = inputs.get<cv::Mat> ("input");
    if (in.channels() == 3)
      cv::split(in, channels_);
    else if (in.channels() == 1)
    {
      for (int i = 0; i < 3; i++)
      {
        channels_[i] = in;
      }
    }
    else
    {
      throw std::runtime_error("unsupported number of channels! must be 1 or 3");
    }
    outputs.get<cv::Mat> ("out_0") = channels_[0];
    outputs.get<cv::Mat> ("out_1") = channels_[1];
    outputs.get<cv::Mat> ("out_2") = channels_[2];
  }

  cv::Mat channels_[3];
};

struct Sobel: ecto::module_interface
{
  Sobel() :
    x_(1), y_(1)
  {

  }

  void initialize(tendrils& p)
  {
    p.declare<int> ("x", "The derivative order in the x direction", 0);
    p.declare<int> ("y", "The derivative order in the y direction", 0);
  }

  void configure(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    SHOW();
    inputs.declare<cv::Mat> ("input", "image.");
    outputs.declare<cv::Mat> ("out", "sobel image");
    x_ = params.get<int> ("x");
    y_ = params.get<int> ("y");
  }
  void process(const tendrils& params, const tendrils& inputs,
      tendrils& outputs)
  {
    SHOW();
    cv::Sobel(inputs.get<cv::Mat> ("input"), outputs.get<cv::Mat> ("out"),
        CV_32F, x_, y_);
  }
  int x_, y_;
};

template<typename T>
struct Adder: ecto::module_interface
{
  Adder()
  {

  }
  void configure(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<T> ("a", "to add to b");
    inputs.declare<T> ("b", "to add to a");
    outputs.declare<T> ("out", "a + b");
  }
  void process(const tendrils& params, const tendrils& inputs,
      tendrils& outputs)
  {
    SHOW();
    outputs.get<T> ("out") = inputs.get<T> ("a") + inputs.get<T> ("b");
  }
};

struct AbsNormalized: ecto::module_interface
{
  AbsNormalized()
  {

  }
  void configure(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("input", "image.");
    outputs.declare<cv::Mat> ("out", "absolute and normalized");
  }
  void process(const tendrils& params, const tendrils& inputs,
      tendrils& outputs)
  {
    SHOW();
    const cv::Mat& m = inputs.get<cv::Mat> ("input");
    cv::Mat & out = outputs.get<cv::Mat> ("out");
    out = cv::abs(m) / (cv::norm(m, cv::NORM_INF) * 0.5);
  }
};

BOOST_PYTHON_MODULE(imgproc)
{
  ecto::wrap<AbsNormalized>("AbsNormalized");
  ecto::wrap<Sobel>("Sobel");
  ecto::wrap<cvtColor>("cvtColor");
  ecto::wrap<Adder<cv::Mat> >("ImageAdder");
  ecto::wrap<ChannelSplitter>("ChannelSplitter");
}
