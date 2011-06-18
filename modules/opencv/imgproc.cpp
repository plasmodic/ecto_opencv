#include <ecto/ecto.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>


using ecto::tendrils;

struct cvtColor
{
  static void declare_params(ecto::tendrils& p)
  {
    std::stringstream ss;
    ss << "Convert an image's color using opencv, possible flags are:\n" << " RGB2GRAY = " << CV_RGB2GRAY << "\n"
        << " RGB2BGR = " << CV_RGB2BGR << "\n" << " RGB2LAB = " << CV_RGB2Lab << "\n" << " BGR2LAB = " << CV_BGR2Lab;
    p.declare<int> ("flag", ss.str(), CV_RGB2BGR);
  }
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("input", "Color image.");
    outputs.declare<cv::Mat> ("out", "input as a Gray image.");
  }
  void configure(tendrils& p, tendrils& inputs, tendrils& outputs)
  {
    flag_ = p.get<int> ("flag");
  }
  int process(const tendrils& inputs, tendrils& outputs)
  {
    cv::cvtColor(inputs.get<cv::Mat> ("input"), outputs.get<cv::Mat> ("out"), flag_);
    return 0;
  }

  int flag_;
};

struct Scale
{
  static void declare_params(ecto::tendrils& p)
  {
    p.declare<float> ("factor","Scale the given image by the constant given", 1.0f);
  }
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("input", "An image");
    outputs.declare<cv::Mat> ("output", "The scaled result.");
  }
  void configure(tendrils& p, tendrils& inputs, tendrils& outputs)
  {
    factor = p.get<float> ("factor");
  }
  int process(const tendrils& inputs, tendrils& outputs)
  {
    outputs.get<cv::Mat> ("output") = inputs.get<cv::Mat> ("input");//, , flag_);
    return 0;
  }
  float factor;
};
struct ChannelSplitter
{
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("input", "The 3 channel image to split.");
    outputs.declare<cv::Mat> ("out_0", "Channel 0.");
    outputs.declare<cv::Mat> ("out_1", "Channel 1.");
    outputs.declare<cv::Mat> ("out_2", "Channel 2.");
  }
  int process(const tendrils& inputs, tendrils& outputs)
  {
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
    return 0;
  }
  cv::Mat channels_[3];
};
struct GaussianBlur
{
  static void declare_params(tendrils& p)
  {
    p.declare<int> ("kernel", "kernel size, if zero computed from sigma", 0);
    p.declare<double> ("sigma", "The first sigma in the guassian.", 1.0);

  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("input", "image.");
    outputs.declare<cv::Mat> ("out", "blurred image");
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    kernel_ = params.get<int> ("kernel");
    sigma_ = params.get<double>("sigma");
  }
  int process(const tendrils& inputs, tendrils& outputs)
  {
    cv::GaussianBlur(inputs.get<cv::Mat> ("input"), outputs.get<cv::Mat> ("out"), cv::Size(kernel_,kernel_),sigma_);
    return 0;
  }
  int kernel_;
  double sigma_;
};


struct Sobel
{
  Sobel() :
    x_(1), y_(1)
  {

  }

  static void declare_params(tendrils& p)
  {
    p.declare<int> ("x", "The derivative order in the x direction", 0);
    p.declare<int> ("y", "The derivative order in the y direction", 0);
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("input", "image.");
    outputs.declare<cv::Mat> ("out", "sobel image");
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    x_ = params.get<int> ("x");
    y_ = params.get<int> ("y");
  }
  int process(const tendrils& inputs, tendrils& outputs)
  {
    cv::Sobel(inputs.get<cv::Mat> ("input"), outputs.get<cv::Mat> ("out"), CV_32F, x_, y_);
    return 0;
  }
  int x_, y_;
};

struct Scharr
{
  Scharr() :
    x_(1), y_(1)
  {

  }

  static void declare_params(tendrils& p)
  {
    p.declare<int> ("x", "The derivative order in the x direction", 0);
    p.declare<int> ("y", "The derivative order in the y direction", 0);
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("input", "image.");
    outputs.declare<cv::Mat> ("out", "scharr image");
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    x_ = params.get<int> ("x");
    y_ = params.get<int> ("y");
  }
  int process(const tendrils& inputs, tendrils& outputs)
  {
    cv::Scharr(inputs.get<cv::Mat> ("input"), outputs.get<cv::Mat> ("out"), CV_32F, x_, y_);
    return 0;
  }
  int x_, y_;
};

struct CartToPolar
{

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat>("x","x derivative image.");
    inputs.declare<cv::Mat>("y","y derivative image.");
    outputs.declare<cv::Mat>("angle","The angle image.");
    outputs.declare<cv::Mat>("magnitude","The magnitude image.");
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    cv::Mat x = inputs.get<cv::Mat> ("x"),y = inputs.get<cv::Mat>("y");
    cv::Mat& angle = outputs.get<cv::Mat> ("angle");
    cv::Mat& magnitude = outputs.get<cv::Mat>("magnitude");
    cv::cartToPolar(x,y,magnitude,angle,true);
    return 0;
  }
};

struct KMeansGradient
{

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat>("x","x derivative image.");
    inputs.declare<cv::Mat>("y","y derivative image.");
    outputs.declare<cv::Mat>("angle","The angle image.");
    outputs.declare<cv::Mat>("magnitude","The magnitude image.");
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    cv::Mat x = inputs.get<cv::Mat> ("x"),y = inputs.get<cv::Mat>("y");
    cv::Mat gradient_[] = {x,y};
    cv::Mat gradient;
    cv::merge(gradient_,2,gradient);
    cv::Mat labels,centers;
    cv::kmeans(gradient.reshape(2,x.size().area()),20,labels,cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 100, 1.0),3, cv::KMEANS_RANDOM_CENTERS,centers);
    labels = labels.reshape(1,x.rows);
    std::cout << labels.size().width << " "<< labels.size().height << std::endl;
    std::cout << centers << std::endl;
    labels.convertTo(outputs.get<cv::Mat>("angle"),CV_8UC1);
    outputs.get<cv::Mat>("angle") *= 255/20.0;
    return 0;
  }
};

template<typename T>
struct Adder
{
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<T> ("a", "to add to b");
    inputs.declare<T> ("b", "to add to a");
    outputs.declare<T> ("out", "a + b");
  }
  int process(const tendrils& inputs, tendrils& outputs)
  {
    outputs.get<T> ("out") = inputs.get<T> ("a") + inputs.get<T> ("b");
    return 0;
  }
};

struct BitwiseAnd
{
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("a", "to and to b");
    inputs.declare<cv::Mat> ("b", "to and to a");
    outputs.declare<cv::Mat> ("out", "a + b");
  }
  int process(const tendrils& inputs, tendrils& outputs)
  {
    cv::Mat a =inputs.get<cv::Mat>("a"),b = inputs.get<cv::Mat>("b");

    if(a.empty()) throw std::runtime_error("a is empty");
    if(b.empty()) throw std::runtime_error("b is empty");
    if(a.size() != b.size()) throw std::runtime_error("a.size != b.size");
    cv::bitwise_and(a,b,outputs.get<cv::Mat>("out"));
    return 0;
  }
};

struct AbsNormalized
{
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("input", "image.");
    outputs.declare<cv::Mat> ("out", "absolute and normalized");
  }
  int process(const tendrils& inputs, tendrils& outputs)
  {
    cv::Mat m = inputs.get<cv::Mat> ("input");
    //grab a reference
    cv::Mat & out = outputs.get<cv::Mat> ("out");
    out = cv::abs(m) / (cv::norm(m, cv::NORM_INF) * 0.5);
    return 0;
  }
};

BOOST_PYTHON_MODULE(imgproc)
{
  ecto::wrap<AbsNormalized>("AbsNormalized");
  ecto::wrap<Sobel>("Sobel");
  ecto::wrap<BitwiseAnd>("BitwiseAnd","Bitwise and two matrices");
  ecto::wrap<Scharr>("Scharr", "The scharr operator.");
  ecto::wrap<cvtColor>("cvtColor");
  ecto::wrap<Adder<cv::Mat> >("ImageAdder");
  ecto::wrap<ChannelSplitter>("ChannelSplitter");
  ecto::wrap<CartToPolar>("CartToPolar", "Takes x and y derivatives and does a polar coordinate tranform.");
  ecto::wrap<KMeansGradient>("KMeansGradient", "Takes x and y derivatives and runs kmeans in 2d vectorspace.");
  ecto::wrap<GaussianBlur>("GaussianBlur","Given an image, blurs it.");
}
