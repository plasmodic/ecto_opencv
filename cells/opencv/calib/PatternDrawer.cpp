#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using ecto::tendrils;
namespace calib
{
  struct PatternDrawer
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<int>("rows", "Number of dots in row direction", 4);
      params.declare<int>("cols", "Number of dots in col direction", 11);
    }

    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare<cv::Mat>("input", "The image to to find a vertical lazer line in.");
      in.declare<std::vector<cv::Point2f> >("points", "Circle pattern points.");
      in.declare<bool>("found", "Found the pattern");
      out.declare<cv::Mat>("out", "Pattern Image");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      grid_size_ = cv::Size(params.get<int>("cols"), params.get<int>("rows"));
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      cv::Mat image;
      in["input"] >> image;
      std::vector<cv::Point2f> points;
      in["points"] >> points;
      bool found;
      in["found"] >> found;

      cv::Mat image_out;
      if(image.channels() == 1)
      {
        //so that we have color...
        cv::cvtColor(image,image_out,CV_GRAY2RGB);
      }else
      {
        image_out = image.clone(); // clone our image so we don't corrupt the input.
      }
      if (found)
        cv::drawChessboardCorners(image_out, grid_size_, points, found);
      out["out"] << image_out;
      return 0;
    }
    cv::Size grid_size_;
  };
}
ECTO_CELL(calib, calib::PatternDrawer, "PatternDrawer", "draw pattern");

