#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/filesystem.hpp>
#include <algorithm>

namespace fs = boost::filesystem;
using ecto::tendrils;

struct ButtonProjector
{
  typedef std::vector<cv::Point3f> pts3d_t;
  typedef std::vector<cv::Point2f> pts2d_t;

  static void
  declare_params(tendrils& params)
  {
    params.declare<int>("x", "Button x position.", 640 / 2);
    params.declare<int>("y", "Button y position.", 480 / 2);
    params.declare<int>("radius", "Radius in pixels of the button.", 50);
    params.declare<int>("image_width", "The width of image", 640);
    params.declare<int>("image_height", "The height of image", 480);
  }

  static void
  declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    out.declare<cv::Mat>("button_image", "An image.");
    out.declare<cv::Mat>("mask", "A mask for the button.");
    out.declare<pts2d_t>("points", "The button centroids.");
  }

  void
  configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    cv::Size image_size;
    params["image_width"] >> image_size.width;
    params["image_height"] >> image_size.height;
    cv::Mat image_out = cv::Mat::zeros(image_size, CV_8UC3);
    cv::Mat mask = cv::Mat::zeros(image_size, CV_8UC1);

    pts2d_t points_out;

    int button_x, button_y;
    params["x"] >> button_x;
    params["y"] >> button_y;

    int radius;
    params["radius"] >> radius;


    cv::Point tl = cv::Point(button_x - radius, button_y - radius);
    cv::Point br = cv::Point(button_x + radius, button_y + radius);

    cv::Scalar box_color(255, 0, 0);
    cv::rectangle(image_out, tl, br, box_color, -1, 8);
    cv::circle(image_out, cv::Point(button_x,button_y), radius, cv::Scalar::all(255), -1, 8);
    cv::circle(mask, cv::Point(button_x,button_y), radius, cv::Scalar::all(255), -1, 8);

    outputs["button_image"] << image_out;
    outputs["points"] << points_out;
    outputs["mask"] << mask;

  }

};

ECTO_CELL(projector, ButtonProjector, "ButtonProjector", "Draws buttons.");
