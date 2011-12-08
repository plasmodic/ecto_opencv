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
    typedef ImageSaver C;
    static void
    declare_params(tendrils& params)
    {
      params.declare(&C::filename_format, "filename_format", "The filename format string. "
                     "Must accept one integer, %d. This integer will monotonically increase. "
                     "The extension determines the image format to write.",
                     "./image_%04d.png");
      params.declare(&C::filename, "filename", "A single filename, set this for single file output.", "");
      params.declare(&C::count, "start",
                     "The starting integer value, that will be inserted into the filename format string", 0);
    }

    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      std::string format;
      params["filename_format"] >> format;
      //throw an error on bad format string
      boost::format(format) % 1;
      in.declare(&C::image, "image", "The image to save.").required(true);
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      std::string name;
      if (filename->empty())
      {
        name = boost::str(boost::format(*filename_format) % (*count)++);
      }
      else
      {
        name = *filename;
      }
      std::cout << "Saving image to : " << name << std::endl;
      cv::imwrite(name, *image);
      return ecto::OK;
    }
    ecto::spore<cv::Mat> image;
    ecto::spore<std::string> filename_format, filename;
    ecto::spore<int> count;
  };
}
ECTO_CELL(highgui, ecto_opencv::ImageSaver, "ImageSaver", "An file saver for images.");
