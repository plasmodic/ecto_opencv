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
      params.declare<std::string>("filename_format", "The filename format string. "
                                  "Must accept one integer, %d. This integer will monotonically in crease. "
                                  "The extension determines the image format to write.",
                                  "./image_%04d.png");
      params.declare<int>("start", "The starting integer value, that will be inserted into the filename format string",
                          0);
    }

    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      std::string format;
      params["filename_format"] >> format;
      //throw an error on bad format string
      boost::format(format) % 1;
      in.declare<cv::Mat>("image", "The original image to draw the pose onto.").required(true);
    }
    void
    configure(const tendrils&p, const tendrils&in, const tendrils&o)
    {
      filename_format = p["filename_format"];
      count = p["start"];
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      cv::Mat image;
      in["image"] >> image;
      std::string filename = boost::str(boost::format(*filename_format) % (*count)++);
      std::cout << "Saving image to : " << filename << std::endl;
      cv::imwrite(filename, image);
      return ecto::OK;
    }

    ecto::spore<std::string> filename_format;
    ecto::spore<int> count;
  };
}
ECTO_CELL(highgui, ecto_opencv::ImageSaver, "ImageSaver", "An file saver for images.");
