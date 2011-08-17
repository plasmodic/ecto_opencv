#include <ecto/ecto.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <boost/format.hpp>
#define BOOST_FILESYSTEM_VERSION 2
#include <boost/filesystem.hpp>

#include <iostream>
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
      params.declare<std::string>("ext", "The image extension to look for.", ".png|.jpg|.bmp");
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      //set outputs
      outputs.declare<cv::Mat>("out", "A video frame.", cv::Mat());
      outputs.declare<int>("frame_number", "The number of frames captured.", 0);
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      path = params.get<std::string>("path");
      ext = params.get<std::string>("ext");
      fs::path x(path);
      if (!fs::is_directory(x))
        throw std::runtime_error(path + " is not a directory");

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
            //std::cout << x.string() << "\n";
            images.push_back(x.string());
          }
        } catch (const std::exception &)
        {
          //std::cout << dir_itr->filename() << " " << ex.what() << std::endl;
        }
      }
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      if (images.empty())
        return 1;
      //outputs.get is a reference;
      outputs.get<cv::Mat>("out") = cv::imread(images.back());
      images.pop_back();
      //increment our frame number.
      ++(outputs.get<int>("frame_number"));
      return 0;
    }
    std::string path;
    std::string ext;
    std::vector<std::string> images;

  };
}
ECTO_CELL(highgui, ecto_opencv::ImageReader, "ImageReader", "Read images from a directory.");
