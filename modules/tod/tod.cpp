#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
using ecto::tendrils;
//#include "opencv_candidate/PoseRT.h"
/* BOILER_PLATE_MODULE
 struct MyModule
 {
 static void declare_params(tendrils& params);
 static void declare_io(const tendrils& params, tendrils& in, tendrils& out);
 void configure(tendrils& params);
 int process(const tendrils& in, tendrils& out);
 void destroy();
 };
 */

namespace tod
{

struct PlanarSegmentation
{
  static void declare_params(tendrils& params)
  {
    params.declare<float> (
        "x_crop",
        "The amount to keep in the x direction (meters) relative to the coordinate frame defined by the pose.",
        0.25);
    params.declare<float> (
        "y_crop",
        "The amount to keep in the y direction (meters) relative to the coordinate frame defined by the pose.",
        0.25);
    params.declare<float> (
        "z_crop",
        "The amount to keep in the z direction (meters) relative to the coordinate frame defined by the pose.",
        0.25);
  }

  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    in.declare<cv::Mat> ("depth", "The depth image to segment");
    //FIXME use a pose object here?
    in.declare<cv::Mat> ("R", "The pose rotation matrix");
    in.declare<cv::Mat> ("T", "The pose traslation vector");
    out.declare<cv::Mat> ("mask",
        "The output mask, determined by the segmentation. "
          "255 is the value for objects satisfying the constraints. 0 otherwise.");
  }
  void configure(tendrils& params)
  {

  }
  int process(const tendrils& in, tendrils& out)
  {
    return 0;
  }
};

}

BOOST_PYTHON_MODULE(tod)
{
  ecto::wrap<tod::PlanarSegmentation>("PlanarSegmentation", "Given a pose, "
      "assuming it describes the center of the object coordinate system and "
      "lies on a plane, segment the object from the plane");
}
