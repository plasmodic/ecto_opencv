#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>

#include "calib.hpp"

using ecto::tendrils;
namespace calib
{
  struct CameraIntrinsics
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<std::string>("camera_file", "The camera calibration file. Typically a .yml", "camera.yml");
    }
    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      out.declare<cv::Size>("image_size", "The image size.");
      out.declare<cv::Mat>("K", "3x3 camera intrinsic matrix.");
      out.declare<cv::Mat>("D", "The distortion vector.");
      out.declare<std::string>("camera_model", "The camera model. e.g pinhole,...", "pinhole");
    }
    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& out)
    {
      readOpenCVCalibration(camera, params.get<std::string>("camera_file"));
      out.get<cv::Mat>("K") = camera.K;
      out.get<cv::Mat>("D") = camera.D;
      out.get<cv::Size>("image_size") = camera.image_size;
    }
    Camera camera;
  };
}

ECTO_CELL(calib, calib::CameraIntrinsics, "CameraIntrinsics",
          "This reads a camera calibration file and puts the results on the outputs.");
