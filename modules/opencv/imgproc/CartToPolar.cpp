#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using ecto::tendrils;
using ecto::spore;

namespace imgproc
{
  struct CartToPolar
  {

    static void
    declare_io(const tendrils& p, tendrils& i, tendrils& o)
    {
      i.declare<cv::Mat>("x", "x derivative image.");
      i.declare<cv::Mat>("y", "y derivative image.");
      o.declare<cv::Mat>("angle", "The angle image.");
      o.declare<cv::Mat>("magnitude", "The magnitude image.");
    }

    void
    configure(const tendrils& p, const tendrils& i, const tendrils& o)
    {
      x = i["x"];
      y = i["y"];
      angle = o["angle"];
      magnitude = o["magnitude"];
    }
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      *angle = cv::Mat();
      *magnitude = cv::Mat();
      cv::cartToPolar(*x, *y, *magnitude, *angle, true);
      return 0;
    }
    spore<cv::Mat> x, y, angle, magnitude;
  };

}
ECTO_CELL(imgproc, imgproc::CartToPolar, "CartToPolar", "Applies a cartesian to polor transform.");

