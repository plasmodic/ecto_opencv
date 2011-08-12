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
  struct ImageSaver
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<std::string>("filename", "The filename prefix to save the image to.", "./image_");
    }
    ImageSaver()
        :
          count(0)
    {
    }
    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare<cv::Mat>("image", "The original image to draw the pose onto.");
      in.declare<int>("trigger", "'s' to save.", 0);
    }
    void
    configure(const tendrils&p, const tendrils&in, const tendrils&o)
    {
      prefix = p["filename"];
    }
    int
    process(const tendrils& in, const tendrils& out)
    {
      int trigger;
      in["trigger"] >> trigger;
      if (trigger != 's')
      {
        return 0;
      }
      cv::Mat image;
      in["image"] >> image;
      std::string filename = boost::str(boost::format("%s%04d.png") % *prefix % count++);
      std::cout << "Saving image to : " << filename << std::endl;
      cv::imwrite(filename, image);
      return 0;
    }
    int count;
    ecto::spore<std::string> prefix;
  };
}
ECTO_CELL(ecto_opencv, ecto_opencv::ImageSaver, "ImageSaver", "An file saver for images.");
