#include <ecto/ecto.hpp>
#include <boost/format.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>
using ecto::tendrils;
namespace ecto_opencv
{
  struct MatWriter
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare(&MatWriter::name, "filename", "Name of mat to write.").required(true);
    }
    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare(&MatWriter::mat, "mat", "A mat to print.").required(true);
    }
    int
    process(const tendrils& in, const tendrils& out)
    {
      cv::FileStorage fs(*name, cv::FileStorage::WRITE);
      fs << "data" << *mat;
      return ecto::OK;
    }
    ecto::spore<std::string> name;
    ecto::spore<cv::Mat> mat;
  };
  struct MatReader
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare(&MatReader::name, "filename", "Name of mat to read.").required(true);
    }
    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      out.declare(&MatReader::mat, "mat", "A mat to print.");
    }
    void
    on_name_change(const std::string& name)
    {
      std::cout << "Reading : " << name << std::endl;
      cv::FileStorage fs(name, cv::FileStorage::READ);

      if (fs.isOpened())
        fs["data"] >> *mat;
      else
        throw std::runtime_error(boost::str(boost::format("%s could not be opened.") % name));
    }
    void
    configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      name.set_callback(boost::bind(&MatReader::on_name_change, this, _1));
      if (name->length())
      {
        name.dirty(true);
        name.notify();
      }
    }
    ecto::spore<std::string> name;
    ecto::spore<cv::Mat> mat;
  };
}

ECTO_CELL(highgui, ecto_opencv::MatWriter, "MatWriter", "Write a cv::Mat to a yaml or xml file.");
ECTO_CELL(highgui, ecto_opencv::MatReader, "MatReader", "Read a cv::Mat from a yaml or xml file.");
