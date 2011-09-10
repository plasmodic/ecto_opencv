#include <ecto/ecto.hpp>

#include <opencv2/highgui/highgui.hpp>
#include "capture_interface.hpp"
#include <boost/format.hpp>
#define BOOST_FILESYSTEM_VERSION 2
#include <boost/filesystem.hpp>

#include <iostream>
#include <algorithm>
#include <iterator>

#include <string>
using ecto::tendrils;

namespace pt = boost::posix_time;
namespace fs = boost::filesystem;
namespace ecto_opencv
{
  namespace
  {
    const char * DEFAULT_EXT = ".png|.jpg|.bmp";
  }

  struct ImageReader
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<std::string>("path", "The path to read images from.", "/tmp/ecto/rules");
      params.declare<std::string>("ext", "The image extension to look for.", DEFAULT_EXT);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      //set outputs
      declare_video_device_outputs(outputs);
    }

    void
    reset_list(const std::string& path, const std::string& ext)
    {
      fs::path x(path);
      if (!fs::is_directory(x))
        throw std::runtime_error(path + " is not a directory");
      std::cout << "looking in " << x.string() << std::endl;
      images.clear();
      fs::directory_iterator end_iter;
      for (fs::directory_iterator dir_itr(path); dir_itr != end_iter; ++dir_itr)
      {
        try
        {
          if (fs::is_regular_file(dir_itr->status()))
          {
            fs::path x(*dir_itr);
            std::string x_ext(x.extension());
            //TODO make this more fancy...
            if (x_ext.size() == 0 || ext.find(x_ext) == std::string::npos)
              continue;
            images.push_back(x.string());
          }
        } catch (const std::exception &)
        {
          //std::cout << dir_itr->filename() << " " << ex.what() << std::endl;
        }
      }
      std::sort(images.rbegin(), images.rend()); //lexographic order.
      std::cout << "Will read the following images in lexographic order:\n";
      std::copy(images.rbegin(), images.rend(), std::ostream_iterator<std::string>(std::cout, " "));
      std::cout << std::endl;
    }
    void
    path_change(const std::string& path)
    {
      if (path != this->path)
        update_list = true;
      this->path = path;
    }

    void
    ext_change(const std::string& ext)
    {
      if (ext != this->ext)
        update_list = true;
      this->ext = ext;
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      update_list = true;
      ext = DEFAULT_EXT; //default
      params["path"]->set_callback<std::string>(boost::bind(&ImageReader::path_change, this, _1));
      params["ext"]->set_callback<std::string>(boost::bind(&ImageReader::ext_change, this, _1));
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      if (update_list)
      {
        update_list = false;
        reset_list(path, ext);
      }
      if (images.empty())
        return ecto::QUIT;
      //outputs.get is a reference;
      outputs["image"] << cv::imread(images.back());
      images.pop_back();
      //increment our frame number.
      ++(outputs.get<int>("frame_number"));
      return 0;
    }
    std::string path, ext;
    bool update_list;
    std::vector<std::string> images;
  };
}
ECTO_CELL(highgui, ecto_opencv::ImageReader, "ImageReader", "Read images from a directory.");
