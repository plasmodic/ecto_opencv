#include <ecto/ecto.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#if CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION == 4 && CV_SUBMINOR_VERSION < 10
#include <cv_backports/imshow.hpp>
#else
namespace cv_backports {
  using cv::destroyWindow;
  using cv::imshow;
  using cv::namedWindow;
  using cv::setWindowProperty;
  using cv::startWindowThread;
  using cv::waitKey;
}
#endif

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
  struct CloseWindow
  {
    CloseWindow(const std::string& name)
        :
          name(name)
    {
    }
    void
    operator()(const boost::signals2::connection& c) const
    {
      c.disconnect();
      cv_backports::destroyWindow(name);
    }
    std::string name;
  };

  struct ImshowJob
  {
    ImshowJob(const cv::Mat& image, const std::string& name, bool full_screen, bool auto_size)
        :
          image(image),
          name(name),
          full_screen(full_screen),
          auto_size(auto_size)
    {
    }

    void
    operator()(const boost::signals2::connection& c) const
    {
      c.disconnect();
      if (full_screen)
      {
        cv_backports::namedWindow(name, CV_WINDOW_KEEPRATIO);
        cv_backports::setWindowProperty(name, CV_WND_PROP_FULLSCREEN, true);
      }
      else if (auto_size)
      {
        cv_backports::namedWindow(name, CV_WINDOW_KEEPRATIO);
      }
      cv_backports::imshow(name, image);
    }
    const cv::Mat image;
    std::string name;
    bool full_screen, auto_size;
  };

  struct HighGuiRunner
  {
    typedef boost::signals2::signal<void(void)> sig_type;
    typedef boost::function<void(const boost::signals2::connection&)> jop_type;

    HighGuiRunner()
        :
          lastKey(0xff)
    {
      t.reset(new boost::thread(boost::ref(*this)));
    }

    ~HighGuiRunner()
    {
      t->interrupt();
      t->join();
      t.reset();
    }

    void
    operator()()
    {
      cv_backports::startWindowThread();
      while (!boost::this_thread::interruption_requested())
      {
        jobs();
        lastKey = 0xff & cv_backports::waitKey(10);
        keys[lastKey] = true;
      }
    }

    void
    post_job(const jop_type& s)
    {
      sig_type::extended_slot_type job(s);
      jobs.connect_extended(job);
    }

    bool
    testKey(int time, unsigned char key, bool reset)
    {
      if (time > 0)
      {
        int count = 0;
        while (lastKey == 0xff && count++ < time)
        {
          boost::this_thread::sleep(boost::posix_time::millisec(1));
        }
      }
      else if (time == 0)
      {
        while (lastKey == 0xff)
        {
          boost::this_thread::sleep(boost::posix_time::millisec(1));
        }
      }
      bool pressed = keys[key];
      if (reset)
        keys[key] = false;
      return pressed;
    }
    unsigned char lastKey;
    boost::shared_ptr<boost::thread> t;
    sig_type jobs;
    std::bitset<0xFF> keys;
  }
  ;

  namespace
  {
    boost::shared_ptr<HighGuiRunner> runner;
  }
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
      inputs.declare<cv::Mat>("image", "The image to show").required(true);
      bp::object triggers;
      params["triggers"] >> triggers;
      if (!triggers || triggers == bp::object())
        return; //no user supplied triggers.

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
      ECTO_SCOPED_CALLPYTHON();
      window_name_ = params.get<std::string>("name");
      waitkey_ = params.get<int>("waitKey");
      auto_size_ = params.get<bool>("autoSize");
      full_screen_ = params["maximize"];
      image_ = inputs["image"];

      bp::object triggers;
      params["triggers"] >> triggers;
      if (!triggers || triggers == bp::object())
        return; //no user supplied triggers.

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
      if (!runner)
      {
        runner.reset(new HighGuiRunner);
      }
      cv::Mat image = *image_;
      std::for_each(trigger_keys_.begin(), trigger_keys_.end(), trigger_reset());
      if (image.empty())
      {
        return ecto::OK;
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

      runner->post_job(ImshowJob(image, window_name_, *full_screen_, auto_size_));

      if (runner->testKey(waitkey_, 'q', true) || runner->testKey(-1, 27, true))
      {
        runner->post_job(CloseWindow(window_name_));
        return ecto::QUIT;
      }
      typedef std::pair<int, ecto::spore<bool> > KeySporeT;
      BOOST_FOREACH(KeySporeT x, trigger_keys_)
          {
            *(x.second) = runner->testKey(-1, x.first, true);
          }
      return ecto::OK;
    }

    ~imshow()
    {
      if(runner)
        runner->post_job(CloseWindow(window_name_));
    }

    std::string window_name_;
    int waitkey_;
    bool auto_size_;
    ecto::spore<bool> full_screen_;
    ecto::spore<cv::Mat> image_;
    std::map<int, ecto::spore<bool> > trigger_keys_;
  };
}
ECTO_THREAD_UNSAFE(ecto_opencv::imshow);
ECTO_CELL(highgui, ecto_opencv::imshow, "imshow", "Displays an image. If a dictionary is defined in triggers, each key "
"is defined as an output, the value being a bool of whether it's been pressed");
