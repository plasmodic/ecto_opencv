#include <ecto/ecto.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "interfaces.h"

/** Interface to cv::drawKeypoints, to draw keypoints to an image
 */
struct DrawKeypoints
{
  typedef std::vector<cv::KeyPoint> kpts_t;
  typedef DrawKeypoints C;
  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare(&C::in_image, "image", "The input image, used as the base to draw on.");
    inputs.declare(&C::in_kpts, "keypoints", "The keypoints to draw.");
    outputs.declare(&C::out_image, "image", "The output image.");
  }

  int
  process(const tendrils& inputs, const tendrils& outputs)
  {
    *out_image = cv::Mat(); //reset the output... mthreaded
    cv::drawKeypoints(*in_image, *in_kpts, *out_image, cv::Scalar(255,0,0));
    return ecto::OK;
  }
  ecto::spore<cv::Mat> in_image, out_image;
  ecto::spore<kpts_t> in_kpts;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ECTO_CELL(features2d, DrawKeypoints, "DrawKeypoints", "Draws keypoints.");
