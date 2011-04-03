#include <ecto/ecto.hpp>

#include <iostream>

#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

//disable show in here
#if 1
#ifdef SHOW
#undef SHOW
#define SHOW() do{}while(false)
#endif
#endif

struct VideoCapture : ecto::module
{
  void Config()
  {
    SHOW();
    int video_device = getParam<int> ("video_device");
    std::string video_file = getParam<std::string> ("video_file");
    capture = cv::VideoCapture();
    if (!video_file.empty())
    {
      capture.open(video_file);
      if (!capture.isOpened())
        throw std::runtime_error("Could not open the video file : " + video_file);
    }
    else
    {
      capture.open(video_device);
      if (!capture.isOpened())
        throw std::runtime_error("Could not open video device : " + video_device);
    }

    //set outputs
    setOut<cv::Mat> ("out", "A video frame.", cv::Mat());
    setOut<int> ("frame_number", "The number of frames captured.", 0);
  }

  static void Params(tendrils_t& params)
  {
    SHOW();
    params.set<int> ("video_device", "The device ID to open.", 0);
    params.set<std::string> ("video_file", "A video file to read, leave empty to open a video device.", "");
  }

  void Process()
  {
    SHOW();
    //getOut is a reference;
    capture >> getOut<cv::Mat> ("out");
    //increment our frame number.
    ++(getOut<int> ("frame_number"));
  }
  cv::VideoCapture capture;

};

struct imshow : ecto::module
{
  static void Params(tendrils_t& params)
  {
    SHOW();
    params.set<std::string> ("name", "The window name", "image");
    params.set<int> ("waitKey", "Number of millis to wait, -1 for not at all, 0 for infinity.", -1);
    params.set<bool> ("autoSize", "Autosize the window.", true);
  }

  void Config()
  {
    SHOW();
    window_name_ = getParam<std::string> ("name");
    waitkey_ = getParam<int> ("waitKey");
    auto_size_ = getParam<bool> ("autoSize");
    setIn<cv::Mat> ("in", "The image to show");
    setOut<int> ("out", "Character pressed.");
  }

  void Process()
  {
    SHOW();
    const cv::Mat& image = getIn<cv::Mat> ("in");
    if (image.empty())
    {
      getOut<int> ("out") = 0;
      throw std::logic_error("empty image!");
      return;
    }

    if (auto_size_)
    {
      cv::namedWindow(window_name_, CV_WINDOW_KEEPRATIO);
    }
    cv::imshow(window_name_, image);
    if (waitkey_ >= 0)
    {
      getOut<int> ("out") = int(0xff & cv::waitKey(waitkey_));
    }
    else
    {
      getOut<int> ("out") = 0;
    }
  }
  std::string window_name_;
  int waitkey_;
  bool auto_size_;
};

ECTO_MODULE(highgui)
{
  ecto::wrap<VideoCapture>("VideoCapture");
  ecto::wrap<imshow>("imshow");
}
