#include <ecto/ecto.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>


using ecto::tendrils;

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
  void configure(const tendrils& p, const tendrils& inputs, const tendrils& outputs)
  {
    factor = p.get<float> ("factor");
  }
  int process(const tendrils& inputs, const tendrils& outputs)
  {
    assert(false && "Scale doesn't appear to actually scale anything");
    outputs.get<cv::Mat> ("output") = inputs.get<cv::Mat> ("input");//, , flag_);
    return 0;
  }
  float factor;
};
ECTO_CELL(imgproc, Scale, "Scale", "Scales an image.");


struct ChannelSplitter
{
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("input", "The 3 channel image to split.");
    outputs.declare<cv::Mat> ("out_0", "Channel 0.");
    outputs.declare<cv::Mat> ("out_1", "Channel 1.");
    outputs.declare<cv::Mat> ("out_2", "Channel 2.");
  }
  int process(const tendrils& inputs, const tendrils& outputs)
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
ECTO_CELL(imgproc, ChannelSplitter, "ChannelSplitter", "Splits an image into 3 channels.");

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

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    kernel_ = params.get<int> ("kernel");
    sigma_ = params.get<double>("sigma");
  }
  int process(const tendrils& inputs, const tendrils& outputs)
  {
    cv::GaussianBlur(inputs.get<cv::Mat> ("input"), outputs.get<cv::Mat> ("out"), cv::Size(kernel_,kernel_),sigma_);
    return 0;
  }
  int kernel_;
  double sigma_;
};
ECTO_CELL(imgproc, GaussianBlur, "GaussianBlur", "Applies a gaussian blur operator");

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

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    x_ = params.get<int> ("x");
    y_ = params.get<int> ("y");
  }
  int process(const tendrils& inputs, const tendrils& outputs)
  {
    cv::Scharr(inputs.get<cv::Mat> ("input"), outputs.get<cv::Mat> ("out"), CV_32F, x_, y_);
    return 0;
  }
  int x_, y_;
};
ECTO_CELL(imgproc, Scharr, "Scharr", "Applies a schar operator");


struct CartToPolar
{

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat>("x","x derivative image.");
    inputs.declare<cv::Mat>("y","y derivative image.");
    outputs.declare<cv::Mat>("angle","The angle image.");
    outputs.declare<cv::Mat>("magnitude","The magnitude image.");
  }

  int process(const tendrils& inputs, const tendrils& outputs)
  {
    cv::Mat x = inputs.get<cv::Mat> ("x"),y = inputs.get<cv::Mat>("y");
    cv::Mat& angle = outputs.get<cv::Mat> ("angle");
    cv::Mat& magnitude = outputs.get<cv::Mat>("magnitude");
    cv::cartToPolar(x,y,magnitude,angle,true);
    return 0;
  }
};
ECTO_CELL(imgproc, CartToPolar, "CartToPolar", "Applies a cartesian to polor transform.");


struct KMeansGradient
{
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat>("x","x derivative image.");
    inputs.declare<cv::Mat>("y","y derivative image.");
    outputs.declare<cv::Mat>("angle","The angle image.");
    outputs.declare<cv::Mat>("magnitude","The magnitude image.");
  }

  int process(const tendrils& inputs, const tendrils& outputs)
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
ECTO_CELL(imgproc, KMeansGradient, "KMeansGradient", "Takes the kmeans of the gradient of an image.");

template<typename T>
struct Adder
{
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<T> ("a", "to add to b");
    inputs.declare<T> ("b", "to add to a");
    outputs.declare<T> ("out", "a + b");
  }
  int process(const tendrils& inputs, const tendrils& outputs)
  {
    cv::Mat out =  inputs.get<T> ("a") + inputs.get<T> ("b");
    outputs["out"] << out;
    return 0;
  }
};
ECTO_CELL(imgproc, Adder<cv::Mat>, "Adder", "Add an image.");

struct BitwiseAnd
{
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("a", "to and to b");
    inputs.declare<cv::Mat> ("b", "to and to a");
    outputs.declare<cv::Mat> ("out", "a + b");
  }
  int process(const tendrils& inputs, const tendrils& outputs)
  {
    cv::Mat a =inputs.get<cv::Mat>("a"),b = inputs.get<cv::Mat>("b");

    if(a.empty()) throw std::runtime_error("a is empty");
    if(b.empty()) throw std::runtime_error("b is empty");
    if(a.size() != b.size()) throw std::runtime_error("a.size != b.size");
    cv::bitwise_and(a,b,outputs.get<cv::Mat>("out"));
    return 0;
  }
};
ECTO_CELL(imgproc, BitwiseAnd, "BitwiseAnd", "Bitwise and the image.");

struct BitwiseNot
{
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("input", "Image to not.").required(true);
    outputs.declare<cv::Mat> ("out", "!input");
  }
  int process(const tendrils& in, const tendrils& out)
  {
    cv::Mat input, output;
    in["input"] >> input;
    cv::bitwise_not(input,output);
    out["out"] << output;
    return 0;
  }
};
ECTO_CELL(imgproc, BitwiseNot, "BitwiseNot", "Bitwise not the image.")


struct AbsNormalized
{
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("input", "image.");
    outputs.declare<cv::Mat> ("out", "absolute and normalized");
  }
  int process(const tendrils& inputs, const tendrils& outputs)
  {
    cv::Mat m = inputs.get<cv::Mat> ("input");
    //grab a reference
    cv::Mat & out = outputs.get<cv::Mat> ("out");
    out = cv::abs(m) / (cv::norm(m, cv::NORM_INF) * 0.5);
    return 0;
  }
};
ECTO_CELL(imgproc, AbsNormalized, "AbsNormalized", "Takes the absolute value of the image and normalizes")

struct Translate
{
  static void declare_params(tendrils& params)
  {
    params.declare<double>("x", "x translation", 0.0);
    params.declare<double>("y", "y translation", 0.0);
    params.declare<double>("z", "z translation", 0.0);
  }

  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    in.declare<cv::Mat> ("in", "3x1 translation vector.");
    out.declare<cv::Mat> ("out", "3x1 Translation vector (itself translated).");
  }

  cv::Mat offset;

  void configure(const tendrils& params, const tendrils& in, const tendrils& out)
  {
    static double t[3] = { params.get<double>("x"), params.get<double>("y"), params.get<double>("z") };

    offset = cv::Mat(3, 1, CV_64F, t);
    std::cout << "IN CONFIGURE OFFSET IS" << offset << "\n";
  }

  int process(const tendrils& in, const tendrils& out)
  {
    cv::Mat tmp = in.get<cv::Mat>("in").clone();
    std::cout << "MAT IS " << tmp << "\n";
    std::cout << "OFFSET IS " << offset << "\n";
    tmp += offset;
    out.get<cv::Mat>("out") = tmp;
    return 0;
  }
};

ECTO_CELL(imgproc, Translate, "Translate", "Translate a 3x1 vector by another 3x1 vector")
