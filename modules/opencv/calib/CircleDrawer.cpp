#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using ecto::tendrils;


struct CircleDrawer
{
  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    in.declare<cv::Mat> ("image", "The image to draw to.");
    in.declare<std::vector<cv::Vec3f> > ("circles",
                                         "Circles to draw, (x,y,radius).");
    out.declare<cv::Mat> ("image", "The image to draw to.");

  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    image_ = inputs.at("image");
    circles_ = inputs.at("circles");
    draw_image_ = outputs.at("image");
  }

  int process(const tendrils& in, tendrils& out)
  {
    const std::vector<cv::Vec3f>& circles = circles_.read();
    *draw_image_ = image_->clone();

    for (size_t i = 0; i < circles.size(); i++)
    {
      cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // draw the circle center
      cv::circle(*draw_image_, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
      // draw the circle outline
      cv::circle(*draw_image_, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
    }
    return 0;
  }

  ecto::spore<cv::Mat> image_, draw_image_;
  ecto::spore<std::vector<cv::Vec3f> > circles_;

};
ECTO_CELL(calib, CircleDrawer, "CircleDrawer", "Draw circles...");

