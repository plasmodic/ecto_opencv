#include <ecto/ecto.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <string>
using ecto::tendrils;

namespace ecto_opencv
{
  typedef std::vector<uint8_t> RgbData;
  typedef std::vector<uint16_t> DepthData;

  typedef boost::shared_ptr<RgbData> RgbDataPtr;
  typedef boost::shared_ptr<DepthData> DepthDataPtr;

  typedef boost::shared_ptr<const RgbData> RgbDataConstPtr;
  typedef boost::shared_ptr<const DepthData> DepthDataConstPtr;

  struct NiConverter
  {

    static void
    declare_params(tendrils& params)
    {
      params.declare<bool>("rescale", "Convert depth to floating point and rescale.", false);
    }
    static void
    declare_io(const tendrils& params, tendrils& i, tendrils& o)
    {
      i.declare<int>("depth_width", "Depth frame width.");
      i.declare<int>("depth_height", "Depth frame height.");
      i.declare<int>("image_width", "Image frame width.");
      i.declare<int>("image_height", "Image frame height.");
      i.declare<int>("image_channels", "Number of image channels.");
      i.declare<DepthDataConstPtr>("depth_buffer");
      i.declare<RgbDataConstPtr>("image_buffer");

      o.declare<cv::Mat>("image");
      o.declare<cv::Mat>("depth");

    }

    void
    configure(const tendrils& p, const tendrils& i, const tendrils& o)
    {
      depth_height = i["depth_height"];
      depth_width = i["depth_width"];
      image_width = i["image_width"];
      image_height = i["image_height"];
      image_channels = i["image_channels"];
      image_buffer = i["image_buffer"];
      depth_buffer = i["depth_buffer"];
      image = o["image"];
      depth = o["depth"];
      rescale = p["rescale"];
    }

    int
    process(const tendrils&, const tendrils&)
    {
      if (*image_buffer)
      {
        int type = CV_8UC(*image_channels);
        uint8_t * data = (uint8_t*) ((*image_buffer)->data());
        cv::Mat temp;
        cv::Mat im_wrapper_(*image_height, *image_width, type, data);
        if (!im_wrapper_.empty() && im_wrapper_.channels() == 3)
        {
          cv::cvtColor(im_wrapper_, temp, CV_RGB2BGR);
        }
        else if (!im_wrapper_.empty() && im_wrapper_.channels() == 2)
        {
          cv::cvtColor(im_wrapper_,temp, CV_YUV420sp2RGB);
        }
        else
        {
          im_wrapper_.copyTo(temp);
        }
        *image = temp;
      }

      if (*depth_buffer)
      {
        cv::Mat temp;
        uint16_t * data = (uint16_t*) ((*depth_buffer)->data());
        cv::Mat im_wrapper_(*depth_height, *depth_width, CV_16UC1, data);
        if (*rescale)
        {
          im_wrapper_.convertTo(temp, CV_32F, 1 / 1000.);
        }
        else
        {
          im_wrapper_.copyTo(temp);
        }
        *depth = temp;
      }
      return ecto::OK;
    }

    ecto::spore<int> depth_width, depth_height, image_width, image_height, image_channels;
    ecto::spore<DepthDataConstPtr> depth_buffer;
    ecto::spore<RgbDataConstPtr> image_buffer;

    ecto::spore<cv::Mat> image, depth;
    ecto::spore<bool> rescale;
  };
}
ECTO_CELL(highgui, ecto_opencv::NiConverter, "NiConverter", "Read images from a directory.");
