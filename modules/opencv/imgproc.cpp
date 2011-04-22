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


struct cvtColor : ecto::module
{
  cvtColor()
  {
  }
  void Config()
  {
    SHOW();
    flag_ = params.get<int> ("flag");
    inputs.declare<cv::Mat> ("input", "Color image.");
    outputs.declare<cv::Mat> ("out", "input as a Gray image.");
  }
  void Process()
  {
    SHOW();
    cv::cvtColor(inputs.get<cv::Mat> ("input"), outputs.get<cv::Mat> ("out"), flag_);
  }
  static void Params(tendrils_t& p)
  {
    SHOW();
    std::stringstream ss;
    ss << "Convert an image's color using opencv, possible flags are:\n"
        << " RGB2GRAY = " << CV_RGB2GRAY << "\n"
        << " RGB2BGR = " << CV_RGB2BGR  << "\n"
        << " RGB2LAB = " << CV_RGB2Lab << "\n"
        << " BGR2LAB = " << CV_BGR2Lab;
    p["flag"].set<int> (ss.str(), CV_RGB2BGR);
  }
  int flag_;
};

struct ChannelSplitter : ecto::module
{
    void Config()
    {
      SHOW();
      inputs.declare<cv::Mat> ("input", "The 3 channel image to split.");
      outputs.declare<cv::Mat> ("out_0", "Channel 0.");
      outputs.declare<cv::Mat> ("out_1", "Channel 1.");
      outputs.declare<cv::Mat> ("out_2", "Channel 2.");
    }
    void Process()
    {
      SHOW();
      const cv::Mat& in = inputs.get<cv::Mat> ("input");
      if (in.channels() == 3)
        cv::split(in, channels_);
      else if (in.channels() == 1)
      {
        for(int i = 0; i < 3; i++)
        {
          channels_[i] = in;
        }
      }else
      {
        throw std::runtime_error("unsupported number of channels! must be 1 or 3");
      }
      outputs.get<cv::Mat>("out_0") = channels_[0];
      outputs.get<cv::Mat>("out_1") = channels_[1];
      outputs.get<cv::Mat>("out_2") = channels_[2];
    }
    static void Params(tendrils_t& p)
    {
      SHOW();
    }
    cv::Mat channels_[3];
  };

struct Sobel : ecto::module
{
  Sobel() :
    x_(1), y_(1)
  {
    inputs.declare<cv::Mat> ("input", "image.");
    outputs.declare<cv::Mat> ("out", "sobel image");
  }

  static void Params(tendrils_t& p)
  {
    p["x"].set<int> ("The derivative order in the x direction", 0);
    p["y"].set<int> ("The derivative order in the y direction", 0);
  }

  void Config()
  {
    SHOW();
    x_ = params.get<int> ("x");
    y_ = params.get<int> ("y");
  }
  void Process()
  {
    SHOW();
    cv::Sobel(inputs.get<cv::Mat> ("input"), outputs.get<cv::Mat> ("out"), CV_32F, x_, y_);
  }
  int x_, y_;
};

template<typename T>
  struct Adder : ecto::module
  {
    Adder()
    {
      inputs.declare<T> ("a", "to add to b");
      inputs.declare<T> ("b", "to add to a");
      outputs.declare<T> ("out", "a + b");
    }
    void Process()
    {
      SHOW();
      outputs.get<T> ("out") = inputs.get<T> ("a") + inputs.get<T> ("b");
    }
    static void Params(tendrils_t& p)
    {
    }
  };

struct AbsNormalized : ecto::module
{
  AbsNormalized()
  {
    inputs.declare<cv::Mat> ("input", "image.");
    outputs.declare<cv::Mat> ("out", "absolute and normalized");
  }
  static void Params(tendrils_t& p)
  {
  }
  void Process()
  {
    SHOW();
    const cv::Mat& m = inputs.get<cv::Mat> ("input");
    cv::Mat& out = outputs.get<cv::Mat> ("out");
    out = cv::abs(m) / (cv::norm(m, cv::NORM_INF) * 0.5);
  }
};

BOOST_PYTHON_MODULE(imgproc)
{
  ecto::wrap<AbsNormalized>("AbsNormalized");
  ecto::wrap<Sobel>("Sobel");
  ecto::wrap<cvtColor>("cvtColor");
  ecto::wrap<Adder<cv::Mat> >("ImageAdder");
  ecto::wrap<ChannelSplitter> ("ChannelSplitter");
}
