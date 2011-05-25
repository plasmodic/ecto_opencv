#include <ecto/ecto.hpp>

#include <iostream>

#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

using ecto::tendrils;

struct VideoCapture
{
  static void declare_params(tendrils& params)
  {
    params.declare<int> ("video_device", "The device ID to open.", 0);
    params.declare<std::string> ("video_file",
        "A video file to read, leave empty to open a video device.", "");
  }

  static void declare_io(const tendrils& params, tendrils& inputs,
      tendrils& outputs)
  {
    //set outputs
    outputs.declare<cv::Mat> ("out", "A video frame.", cv::Mat());
    outputs.declare<int> ("frame_number", "The number of frames captured.", 0);
  }

  void configure(tendrils& params)
  {
    video_device = params.get<int> ("video_device");
    video_file = params.get<std::string> ("video_file");
    capture = cv::VideoCapture();
  }
  void open_video_device()
  {
    if (capture.isOpened())
      return;

    if (!video_file.empty())
    {
      capture.open(video_file);
      if (!capture.isOpened())
        throw std::runtime_error(
            "Could not open the video file : " + video_file);
    }
    else
    {
      capture.open(video_device);
      if (!capture.isOpened())
        throw std::runtime_error(
            "Could not open video device : " + video_device);
    }
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    open_video_device();

    //outputs.get is a reference;
    capture >> outputs.get<cv::Mat> ("out");

    //increment our frame number.
    ++(outputs.get<int> ("frame_number"));
    return 0;
  }
  cv::VideoCapture capture;
  int video_device;
  std::string video_file;

};

struct imshow
{
  static void declare_params(tendrils& params)
  {
    params.declare<std::string> ("name", "The window name", "image");
    params.declare<int> ("waitKey",
        "Number of millis to wait, -1 for not at all, 0 for infinity.", -1);
    params.declare<bool> ("autoSize", "Autosize the window.", true);
  }

  static void declare_io(const tendrils& params, tendrils& inputs,
      tendrils& outputs)
  {

    inputs.declare<cv::Mat> ("input", "The image to show");
    outputs.declare<int> ("out", "Character pressed.");
  }

  void configure(const tendrils& params)
  {
    window_name_ = params.get<std::string> ("name");
    waitkey_ = params.get<int> ("waitKey");
    auto_size_ = params.get<bool> ("autoSize");
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    cv::Mat image = inputs.get<cv::Mat> ("input");
    if (image.empty())
    {
      outputs.get<int> ("out") = 0;
      throw std::logic_error("empty image!");
    }
    if (auto_size_)
    {
      cv::namedWindow(window_name_, CV_WINDOW_KEEPRATIO);
    }
    cv::imshow(window_name_, image);
    if (waitkey_ >= 0)
    {
      outputs.get<int> ("out") = int(0xff & cv::waitKey(waitkey_));
    }
    else
    {
      outputs.get<int> ("out") = 0;
    }
    return 0;
  }
  std::string window_name_;
  int waitkey_;
  bool auto_size_;
};

BOOST_PYTHON_MODULE(highgui)
{
  ecto::wrap<VideoCapture>("VideoCapture",
      "Use to capture video from a camera or video file.");
  ecto::wrap<imshow>("imshow", "Shows an image in a named window.");
}
