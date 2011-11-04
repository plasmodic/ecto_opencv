#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>
using ecto::tendrils;
namespace ecto_opencv
{
  struct MatPrinter
  {

    static void
    declare_params(tendrils& params)
    {
      params.declare<std::string>("name", "Name of mat to print.").required(true);
    }
    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare<cv::Mat>("mat", "A mat to print.").required(true);
    }

    void
    configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      name = params["name"];
      mat = in["mat"];
    }
    int
    process(const tendrils& in, const tendrils& out)
    {
      std::cout << *name << " = " << *mat << std::endl;
      return 0;
    }
    ecto::spore<std::string> name;
    ecto::spore<cv::Mat> mat;
  };
}

ECTO_CELL(highgui, ecto_opencv::MatPrinter, "MatPrinter", "Print a cv::Mat to the console.");
