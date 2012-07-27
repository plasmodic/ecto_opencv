#include<fcntl.h>
#include <sys/file.h>

#include <ecto/ecto.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <string>

using ecto::tendrils;
using ecto::spore;

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
      params.declare(&C::filename_param, "filename_param", "A single filename, set this for single file output.", "");
      params.declare(&C::count, "start",
                     "The starting integer value, that will be inserted into the filename format string", 0);
      params.declare(&C::lock_name_, "lock_name", "If set to something, an flock will be created for that file", "");
    }

    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      if (!filename_format->empty())
      {
        std::string format;
        params["filename_format"] >> format;
        //throw an error on bad format string
        boost::format(format) % 1;
      }

      in.declare(&C::image, "image", "The image to save.").required(true);
      in.declare(&C::filename, "filename", "A single filename, set this for single file output.", "");
      //outputs.
      out.declare(&C::filename_output, "filename", "The filename that was used for saving the last frame.");
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      std::string name;
      if (filename->empty())
      {
        if (filename_format->empty())
          name = *filename_param;
        else
          name = boost::str(boost::format(*filename_format) % (*count)++);
      }
      else
      {
        name = *filename;
      }
      fs::path p(name);
      fs::path parent(p.parent_path());
      if (!fs::is_directory(parent) && !parent.empty())
        fs::create_directory(parent);

      if (!lock_name_->empty())
      {
        int file = open(lock_name_->c_str(), O_WRONLY);
        flock(file, LOCK_EX);
        cv::imwrite(name, *image);
        flock(file, LOCK_UN);
        close(file);
      }
      else
        cv::imwrite(name, *image);
      *filename_output = name;
      return ecto::OK;
    }
    spore<cv::Mat> image;
    spore<std::string> filename_format, filename, filename_param, filename_output;
    spore<int> count;
    spore<std::string> lock_name_;
  };
}
ECTO_CELL(highgui, ecto_opencv::ImageSaver, "ImageSaver", "An file saver for images.");
