#include <fstream>
#include <iostream>
#include <string>

#include <boost/interprocess/sync/file_lock.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include "highgui.h"
using ecto::tendrils;

namespace pt = boost::posix_time;
namespace fs = boost::filesystem;
namespace ecto_opencv
{
  struct imread
  {
    typedef imread C;
    static void
    declare_params(tendrils& params)
    {
      params.declare(&C::image_file, "image_file", "The path to the image to read.", "lena.jpg");
      params.declare(&C::mode, "mode", "The image read mode.", Image::COLOR);
      params.declare(&C::lock_name_, "lock_name",
                     "If set to something, an flock will be created for that file", "");
      params.declare(&C::refresh_, "refresh", "If true, the image is re-read every time", false);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      //set outputs
      outputs.declare(&C::image_out, "image", "The image in full color.", cv::Mat());
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      image_file.set_callback(boost::bind(&imread::filechange_verbose, this, _1));
      image_file.dirty(true);

      // Create the original lock
      fs::path file(*lock_name_);
      if (!fs::exists(file))
      {
        std::fstream file(lock_name_->c_str(), std::fstream::in | std::fstream::out);
        file << "nothing";
        file.close();
      }
    }

    void
    filechange(const std::string& file_name)
    {
      cv::Mat image;
      if (!lock_name_->empty())
      {
        boost::interprocess::file_lock flock(lock_name_->c_str());
        flock.lock();
        image = cv::imread(file_name, *mode);
        flock.unlock();
      }
      else
        image = cv::imread(file_name, *mode);

      *image_out = image;
    }

    void
    filechange_verbose(const std::string& file_name)
    {
      filechange(file_name);

      std::cout << "read image:" << file_name << std::endl;
      std::cout << "width:" << image_out->cols << " height:" << image_out->rows << " channels:" << image_out->channels()
      << std::endl;
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      if (!(*refresh_))
        return ecto::OK;

      filechange(*image_file);
      return ecto::OK;
    }

    ecto::spore<cv::Mat> image_out;
    ecto::spore<Image::Modes> mode;
    ecto::spore<std::string> image_file;
    ecto::spore<std::string> lock_name_;
    ecto::spore<bool> refresh_;
  };
}
ECTO_CELL(highgui, ecto_opencv::imread, "imread", "Reads a single image, const cell.");
