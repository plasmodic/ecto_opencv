#include <ecto/ecto.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <string>
using ecto::tendrils;

namespace pt = boost::posix_time;
namespace fs = boost::filesystem;
namespace bp = boost::python;

namespace
{
  struct trigger_reset
  {
    template<typename Value>
    void
    operator()(Value& x) const
    {
      *(x.second) = false;
    }

  };

}
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
      params.declare<bp::object>("triggers", "A dict of trigger keys, e.g. {'x_key':ord('x')}");
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Mat>("input", "The image to show").required(true);

      outputs.declare<int>("out", "Character pressed."); //optional output.

      bp::object triggers;
      params["triggers"] >> triggers;
      if (!triggers || triggers == bp::object())
        return;//no user supplied triggers.

      if (params.get<int>("waitKey") < 0)
              throw std::runtime_error(
                  "You may not have a waitKey of less than zero when you are supplying triggers."
                  " waitKey is what captures keypress events.");

      bp::list l = bp::dict(triggers).items();
      for (int j = 0, end = bp::len(l); j < end; ++j)
      {
        bp::object key = l[j][0];
        bp::object value = l[j][1];
        int k = bp::extract<int>(value);
        outputs.declare<bool>(bp::extract<std::string>(key),
                              boost::str(boost::format("The '%c' key has been pressed.") % char(k)), false);
      }
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      window_name_ = params.get<std::string>("name");
      waitkey_ = params.get<int>("waitKey");
      auto_size_ = params.get<bool>("autoSize");
      full_screen_ = params["maximize"];
      image_ = inputs["input"];
      key_ = outputs["out"];

      bp::object triggers;
      params["triggers"] >> triggers;
      if (!triggers || triggers == bp::object())
        return; //no user supllied triggers.

      bp::list l = bp::dict(triggers).items();
      for (int j = 0, end = bp::len(l); j < end; ++j)
      {
        bp::object key = l[j][0];
        bp::object value = l[j][1];
        int k = bp::extract<int>(value);
        std::string skey = bp::extract<std::string>(key);
        std::cout << "Listening for key: " << char(k) << " on imshow:" << skey << std::endl;
        trigger_keys_[k] = outputs[skey];
      }
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      cv::Mat image = *image_;
      std::for_each(trigger_keys_.begin(), trigger_keys_.end(), trigger_reset());
      *key_ = 0;
      if (image.empty())
      {
        return 0;
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

      *key_ = r;

      if (r == 27 || r == 'q' || r == 'Q')
      {
        std::cout << "QUIT!\n";
        return ecto::QUIT;
      }
      else
      {
        std::map<int, ecto::spore<bool> >::iterator it = trigger_keys_.find(r);
        if (it != trigger_keys_.end())
          *(it->second) = true;
        return ecto::OK;
      }
    }
    std::string window_name_;
    int waitkey_;
    bool auto_size_;
    ecto::spore<bool> full_screen_;
    ecto::spore<cv::Mat> image_;
    ecto::spore<int> key_;
    std::map<int, ecto::spore<bool> > trigger_keys_;
  };
}
ECTO_THREAD_UNSAFE(ecto_opencv::imshow);
ECTO_CELL(highgui, ecto_opencv::imshow, "imshow", "Read images from a directory.");
