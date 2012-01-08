#include <ecto/ecto.hpp>

#include <opencv2/highgui/highgui.hpp>
#include "capture_interface.hpp"
#include <boost/format.hpp>
#define BOOST_FILESYSTEM_VERSION 2
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/regex.hpp>

#include <iostream>
#include <algorithm>
#include <iterator>

#include <string>
using ecto::tendrils;

namespace pt = boost::posix_time;
namespace fs = boost::filesystem;
namespace ecto_opencv
{
  struct ImageReader
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<std::string>("path", "The path to read images from.", "/tmp/ecto/rules");
      params.declare<std::string>("match", "Use images matching this regex (regex.  not glob.)",  ".*\\.(bmp|jpg|png)");
      params.declare<bool>("loop","Loop over the list",false);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      //set outputs
      declare_video_device_outputs(outputs);
    }

    void
    reset_list(const std::string& path)
    {
      fs::path x(path);
      if (!fs::is_directory(x))
        throw std::runtime_error(path + " is not a directory");
      //std::cout << "looking in " << x.string() << std::endl;
      images.clear();
      fs::directory_iterator end_iter;
      for (fs::directory_iterator dir_itr(path); dir_itr != end_iter; ++dir_itr)
      {
        try
        {
          if (fs::is_regular_file(dir_itr->status()))
          {
            fs::path x(*dir_itr);
            if (boost::regex_match(x.string(), re))
              images.push_back(x.string());
          }
        }
        catch (const std::exception &e)
          {
            std::cout << dir_itr->filename() << " " << e.what() << std::endl;
          }
      }
      std::sort(images.begin(), images.end()); //lexographic order.

      std::cout << "Will read the following images in lexographic order:\n";
      std::copy(images.rbegin(), images.rend(), std::ostream_iterator<std::string>(std::cout, " "));
      std::cout << std::endl;

      iter = images.begin();
      update_list = false;
    }

    void
    path_change(const std::string& path)
    {
      if (path != this->path)
        update_list = true;
      this->path = path;
    }

    void
    re_change(const std::string& s)
    {
      update_list = false;
      if (this->re.str() != s)
        {
          this->re = s.c_str();
          update_list = true;
        }
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      params["loop"] >> loop;
      update_list = true;
      params["path"]->set_callback<std::string>(boost::bind(&ImageReader::path_change, this, _1));
      params["match"]->set_callback<std::string>(boost::bind(&ImageReader::re_change, this, _1));
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      if (update_list)
        reset_list(path);

      if (iter == images.end()) {
        if (loop)
          iter = images.begin();
        else
          return ecto::QUIT;
      }

      //outputs.get is a reference;
      outputs["image"] << cv::imread(*iter);

      //increment our frame number.
      ++(outputs.get<int>("frame_number"));
      ++iter;
      return 0;
    }
    std::string path;
    bool update_list, loop;
    std::vector<std::string> images;
    std::vector<std::string>::iterator iter;
    boost::regex re;
  };
}
ECTO_CELL(highgui, ecto_opencv::ImageReader, "ImageReader", "Read images from a directory.");
