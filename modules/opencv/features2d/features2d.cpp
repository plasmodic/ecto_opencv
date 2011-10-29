#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "interfaces.hpp"


/** Interface to cv::drawKeypoints, to draw keypoints to an image
 */
struct DrawKeypoints
{
  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat>("image", "The input image, to draw over.");
    inputs.declare<std::vector<cv::KeyPoint> >("keypoints", "The keypoints to draw.");
    outputs.declare<cv::Mat>("image", "The output image.");
  }

  int
  process(const tendrils& inputs, const tendrils& outputs)
  {
    cv::Mat image = inputs.get<cv::Mat>("image");
    const std::vector<cv::KeyPoint>& keypoints_in = inputs.get<std::vector<cv::KeyPoint> >("keypoints");
    cv::Mat out_image;
    cv::drawKeypoints(image, keypoints_in, out_image);
    outputs.get<cv::Mat>("image") = out_image;
    return 0;
  }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ECTO_CELL(features2d, DrawKeypoints, "DrawKeypoints", "Draws keypoints.");
