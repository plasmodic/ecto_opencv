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
    params.declare<int>("buttons", "Number of buttons", 3);
    params.declare<int>("radius", "Radius in pixels of the button.", 50);
    params.declare<int>("image_width", "The width of image", 640);
    params.declare<int>("image_height", "The height of image", 480);
  }

  static void
  declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    out.declare<cv::Mat>("button_image", "An image.");
    out.declare<pts2d_t>("points", "The button centroids.");
  }

  void
  configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    cv::Size image_size;
    params["image_width"] >> image_size.width;
    params["image_height"] >> image_size.height;
    cv::Mat image_out = cv::Mat::zeros(image_size, CV_8UC3);
    pts2d_t points_out;
    int buttons;
    params["buttons"] >> buttons;
    int radius;
    params["radius"] >> radius;

    int center_x = image_size.width / 2;

    int half_buttons = buttons / 2;
    int half_way = image_size.height / 2;
    int spacing_y = radius;

    int min_y = half_way - (2 * radius + spacing_y) * (half_buttons);
    int max_y = half_way - (2 * radius + spacing_y) * (half_buttons - buttons + 1);

    cv::Point tl = cv::Point(center_x - 2 * radius, min_y - 2 * radius);
    cv::Point tr = cv::Point(center_x + 2 * radius, min_y - 2 * radius);
    cv::Point br = cv::Point(center_x + 2 * radius, max_y + 2 * radius);
    cv::Point bl = cv::Point(center_x - 2 * radius, max_y + 2 * radius);

    cv::Scalar box_color(255, 0, 0);
    cv::rectangle(image_out, tl, br, box_color, -1, 8);
    for (int i = 0; i < buttons; i++)
    {
      cv::Point p;
      p.x = center_x;
      p.y = half_way - (2 * radius + spacing_y) * (half_buttons - i);
      cv::circle(image_out, p, radius, cv::Scalar::all(255), -1, 8);
      points_out.push_back(p);
    }

    outputs["button_image"] << image_out;
    outputs["points"] << points_out;
  }

};

ECTO_CELL(projector, ButtonProjector, "ButtonProjector", "Draws buttons.");
