#include <ecto/ecto.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <string>
using ecto::tendrils;

namespace pt = boost::posix_time;
namespace fs = boost::filesystem;
namespace ecto_opencv
{
  struct imshow
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<std::string>("name", "The window name", "image");
      params.declare<int>("waitKey", "Number of millis to wait, -1 for not at all, 0 for infinity.", -1);
      params.declare<bool>("autoSize", "Autosize the window.", true);
      params.declare<bool>("maximize", "Fullscreen the window, takes precedence over autoSize.", false);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Mat>("input", "The image to show");
      outputs.declare<int>("out", "Character pressed.");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      window_name_ = params.get<std::string>("name");
      waitkey_ = params.get<int>("waitKey");
      auto_size_ = params.get<bool>("autoSize");
      full_screen_ = params["maximize"];
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      cv::Mat image = inputs.get<cv::Mat>("input");
      if (image.empty())
      {
        outputs.get<int>("out") = 0;
        return 0;
        //throw std::logic_error("empty image!");
      }
      if (*full_screen_)
      {
        cv::namedWindow(window_name_, CV_WINDOW_KEEPRATIO);
        cv::setWindowProperty(window_name_, CV_WND_PROP_FULLSCREEN, true);
      }
      else if (auto_size_)
      {
        cv::namedWindow(window_name_, CV_WINDOW_KEEPRATIO);
      }

      if (image.depth() == CV_32F || image.depth() == CV_64F)
      {
        const float scaleFactor = 100;
        cv::Mat show;
        image.convertTo(show, CV_8UC1, scaleFactor);
        image = show;
      }

      if (image.type() == CV_16UC1)
      {
        const float scaleFactor = 0.05f;
        cv::Mat show;
        image.convertTo(show, CV_8UC1, scaleFactor);
        image = show;
      }

      cv::imshow(window_name_, image);

      int r = 0;
      if (waitkey_ >= 0)
        r = 0xff & cv::waitKey(waitkey_);

      if (r == 27 || r == 'q' || r == 'Q')
      {
        std::cout << "QUIT!\n";
        return 1;
      }
      else
      {
        outputs.get<int>("out") = r;
        return 0;
      }
    }
    std::string window_name_;
    int waitkey_;
    bool auto_size_;
    ecto::spore<bool> full_screen_;
  };
}
ECTO_THREAD_UNSAFE(ecto_opencv::imshow);
ECTO_CELL(highgui, ecto_opencv::imshow, "imshow", "Read images from a directory.");
