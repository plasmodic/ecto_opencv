#include <ecto/ecto.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <string>
using ecto::tendrils;
#include "capture_interface.hpp"
namespace pt = boost::posix_time;
namespace fs = boost::filesystem;
namespace ecto_opencv
{

  struct VideoCapture
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<int>("video_device", "The device ID to open.", 0);
      params.declare<std::string>("video_file", "A video file to read, leave empty to open a video device.", "");
      params.declare<unsigned>("width", "Set width to this after opening device", 640);
      params.declare<unsigned>("height", "Set width to this after opening device", 480);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      //set outputs
      declare_video_device_outputs(outputs);
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      video_device = params.get<int>("video_device");
      video_file = params.get<std::string>("video_file");
      width = params.get<unsigned>("width");
      height = params.get<unsigned>("height");
      capture = cv::VideoCapture();
    }
    void
    open_video_device()
    {
      if (capture.isOpened())
        return;

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
      capture.set(CV_CAP_PROP_FRAME_WIDTH, width);
      capture.set(CV_CAP_PROP_FRAME_HEIGHT, height);
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      open_video_device();
      cv::Mat image;
      capture >> image;
      outputs["image"] << image;

      //increment our frame number.
      ++(outputs.get<int>("frame_number"));
      return 0;
    }
    cv::VideoCapture capture;
    int video_device;
    unsigned width, height;
    std::string video_file;

  };
}
ECTO_CELL(highgui, ecto_opencv::VideoCapture, "VideoCapture", "Read images from a directory.");
