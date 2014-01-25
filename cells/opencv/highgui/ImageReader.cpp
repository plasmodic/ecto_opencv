#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

#include <boost/interprocess/sync/file_lock.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/highgui/highgui.hpp>
#include "capture_interface.hpp"
#include <boost/format.hpp>
#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/regex.hpp>

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
      params.declare<std::string>("match", "Use images matching this regex (regex.  not glob.)",
                                  ".*\\.(bmp|jpg|png)");
      params.declare<bool>("loop","Loop over the list",false);
      params.declare(&ImageReader::file_list, "file_list","A list of images to read.");
      params.declare(&ImageReader::lock_name_, "lock_name",
                     "If set to something, an flock will be created for that file", "");
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      //set outputs
      declare_video_device_outputs(outputs);
      inputs.declare(&ImageReader::step, "step","The set at which to read the images.", 1);

      outputs.declare(&ImageReader::image_file, "image_file", "The current image file being read");
    }

    void
    reset_list(const std::string& path)
    {
      fs::path x(path);
      if (!fs::is_directory(x))
        throw std::runtime_error(path + " is not a directory");
      images.clear();
      fs::recursive_directory_iterator end_iter;
      for (fs::recursive_directory_iterator dir_itr(path); dir_itr != end_iter; ++dir_itr)
      {
        try
        {
          if (fs::is_regular_file(dir_itr->status()))
          {
            fs::path x(*dir_itr);
            if (boost::regex_match(x.string(), re)) {
              images.push_back(x.string());
            }
          }
        }
        catch (const std::exception &e)
          {
            std::cout << dir_itr->path().filename() << " " << e.what() << std::endl;
          }
      }

      if (images.size() == 0)
        BOOST_THROW_EXCEPTION(ecto::except::EctoException() <<
                              ecto::except::diag_msg("No files matched regular expression")
                              );

      std::sort(images.begin(), images.end()); //lexographic order.

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
    list_change(const std::vector<std::string>& l)
    {
      images = l;
      update_list = false;
      iter = images.begin();
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      params["loop"] >> loop;
      file_list.set_callback(boost::bind(&ImageReader::list_change,this,_1));
      params["path"]->set_callback<std::string>(boost::bind(&ImageReader::path_change, this, _1));
      params["match"]->set_callback<std::string>(boost::bind(&ImageReader::re_change, this, _1));
      update_list = true;
      params["path"]->dirty(true);
      params["match"]->dirty(true);

      // Create the original lock
      fs::path file(*lock_name_);
      if (!fs::exists(file))
      {
        std::fstream file(lock_name_->c_str(), std::fstream::in | std::fstream::out);
        file << "nothing";
        file.close();
      }
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      if (update_list && file_list->empty())
        reset_list(path);

      if (iter == images.end()) {
        if (loop)
          iter = images.begin();
        else
          return ecto::QUIT;
      }

      //outputs.get is a reference;
      if (!lock_name_->empty())
      {
        boost::interprocess::file_lock flock(lock_name_->c_str());
        flock.lock();
        outputs["image"] << cv::imread(*iter, CV_LOAD_IMAGE_UNCHANGED);
        flock.unlock();
      }
      else
        outputs["image"] << cv::imread(*iter, CV_LOAD_IMAGE_UNCHANGED);

      *image_file = *iter;
      //increment our frame number.
      ++(outputs.get<int>("frame_number"));
      for(int i = 0; i < *step && iter != images.end(); ++i)
        ++iter;
      return ecto::OK;
    }
    std::string path;
    bool update_list, loop;
    std::vector<std::string> images;
    std::vector<std::string>::iterator iter;
    boost::regex re;
    ecto::spore<int> step;
    ecto::spore<std::string> image_file;
    ecto::spore<std::vector<std::string> > file_list;
    ecto::spore<std::string> lock_name_;
  };
}
ECTO_CELL(highgui, ecto_opencv::ImageReader, "ImageReader", "Read images from a directory.");
