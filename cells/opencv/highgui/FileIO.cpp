#include <ecto/ecto.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
using ecto::tendrils;
namespace ecto_opencv
{
  typedef boost::shared_ptr<std::ostream> ostream_ptr;

  struct ImageJpgWriter
  {
    static void
    declare_params(ecto::tendrils& parameters)
    {
      parameters.declare<ostream_ptr>("file", "A filelike object");
    }
    static void
    declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare<cv::Mat>("image", "An image to write.");
      outputs.declare<ostream_ptr>("file", "A filelike object");
    }
    void
    configure(const tendrils& p, const tendrils& i, const tendrils& o)
    {
      stream_ = p["file"];
      image_ = i["image"];
      output_ = o["file"];
    }
    int
    process(const tendrils& /*inputs*/, const tendrils& /*outputs*/)
    {
      std::vector<uint8_t> buffer;
      cv::imencode(".jpg", *image_, buffer);
      std::ostream& out = **stream_;
      out.seekp(0);
      std::copy(buffer.begin(), buffer.end(), std::ostream_iterator<uint8_t>(out));
      out.flush();
      *output_ = *stream_;
      return ecto::OK;
    }
    ecto::spore<cv::Mat> image_;
    ecto::spore<ostream_ptr> stream_,output_;
  };
}

ECTO_CELL(highgui, ecto_opencv::ImageJpgWriter, "ImageJpgWriter", "Writes jpg to a file like object");
