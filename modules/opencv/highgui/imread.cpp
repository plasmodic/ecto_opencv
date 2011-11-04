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
  struct imread
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<std::string>("image_file", "The path to the image to read.", "lena.jpg");
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      //set outputs
      outputs.declare<cv::Mat>("image", "The image in full color.", cv::Mat());
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      std::string file;
      params["image_file"] >> file;
      cv::Mat image = cv::imread(file, CV_LOAD_IMAGE_UNCHANGED);
      outputs["image"] << image;
      std::cout << "read Image:" << file << std::endl;
      std::cout << image.cols << ":" << image.rows << std::endl;
    }

  };
}
ECTO_CELL(highgui, ecto_opencv::imread, "imread", "Reads a single image, const cell.");
