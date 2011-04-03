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
    flag_ = getParam<int> ("flag");
    setIn<cv::Mat> ("in", "Color image.");
    setOut<cv::Mat> ("out", "input as a Gray image.");
  }
  void Process()
  {
    SHOW();
    cv::cvtColor(getIn<cv::Mat> ("in"), getOut<cv::Mat> ("out"), flag_);
  }
  static void Params(tendrils_t& p)
  {
    SHOW();
    std::stringstream ss;
    ss << "Convert an image's color using opencv, possible flags are:\n"
        << " RGB2GRAY = " << CV_RGB2GRAY << "\n"
        << " RGB2BGR = " << CV_RGB2BGR  << "\n"
        << " RGB2LAB=" << CV_RGB2Lab << "\n"
        << " BGR2LAB=" << CV_BGR2Lab;
    p["flag"].set<int> (ss.str(), CV_RGB2BGR);
  }
  int flag_;
};

struct ChannelSplitter : ecto::module
{
    void Config()
    {
      SHOW();
      setIn<cv::Mat> ("in", "The 3 channel image to split.");
      setOut<cv::Mat> ("out_0", "Channel 0.");
      setOut<cv::Mat> ("out_1", "Channel 1.");
      setOut<cv::Mat> ("out_2", "Channel 2.");
    }
    void Process()
    {
      SHOW();
      const cv::Mat& in = getIn<cv::Mat> ("in");
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
      getOut<cv::Mat>("out_0") = channels_[0];
      getOut<cv::Mat>("out_1") = channels_[1];
      getOut<cv::Mat>("out_2") = channels_[2];
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
    setIn<cv::Mat> ("in", "image.");
    setOut<cv::Mat> ("out", "sobel image");
  }

  static void Params(tendrils_t& p)
  {
    p["x"].set<int> ("The derivative order in the x direction", 0);
    p["y"].set<int> ("The derivative order in the y direction", 0);
  }

  void Config()
  {
    SHOW();
    x_ = getParam<int> ("x");
    y_ = getParam<int> ("y");
  }
  void Process()
  {
    SHOW();
    cv::Sobel(getIn<cv::Mat> ("in"), getOut<cv::Mat> ("out"), CV_32F, x_, y_);
  }
  int x_, y_;
};

template<typename T>
  struct Adder : ecto::module
  {
    Adder()
    {
      setIn<T> ("a", "to add to b");
      setIn<T> ("b", "to add to a");
      setOut<T> ("out", "a + b");
    }
    void Process()
    {
      SHOW();
      getOut<T> ("out") = getIn<T> ("a") + getIn<T> ("b");
    }
    static void Params(tendrils_t& p)
    {
    }
  };

struct AbsNormalized : ecto::module
{
  AbsNormalized()
  {
    setIn<cv::Mat> ("in", "image.");
    setOut<cv::Mat> ("out", "absolute and normalized");
  }
  static void Params(tendrils_t& p)
  {
  }
  void Process()
  {
    SHOW();
    const cv::Mat& m = getIn<cv::Mat> ("in");
    cv::Mat& out = getOut<cv::Mat> ("out");
    out = cv::abs(m) / (cv::norm(m, cv::NORM_INF) * 0.5);
  }
};

ECTO_MODULE(imgproc)
{
  ecto::wrap<AbsNormalized>("AbsNormalized");
  ecto::wrap<Sobel>("Sobel");
  ecto::wrap<cvtColor>("cvtColor");
  ecto::wrap<Adder<cv::Mat> >("ImageAdder");
  ecto::wrap<ChannelSplitter> ("ChannelSplitter");
}
