#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "imgproc.h"
#include <iostream>

using ecto::tendrils;
namespace imgproc
{
  struct Translate
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<double>("x", "x translation", 0.0);
      params.declare<double>("y", "y translation", 0.0);
      params.declare<double>("z", "z translation", 0.0);
    }

    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare<cv::Mat>("in", "3x1 translation vector.");
      out.declare<cv::Mat>("out", "3x1 Translation vector (itself translated).");
    }

    cv::Mat offset;

    void
    configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      static double t[3] =
      { params.get<double>("x"), params.get<double>("y"), params.get<double>("z") };
      offset = cv::Mat(3, 1, CV_64F, t);
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      cv::Mat tmp = in.get<cv::Mat>("in").clone();
      tmp += offset;
      out.get<cv::Mat>("out") = tmp;
      return 0;
    }
  };

}

ECTO_CELL(imgproc, imgproc::Translate, "Translate", "Translate a 3x1 vector by another 3x1 vector")
