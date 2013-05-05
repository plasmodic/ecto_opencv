#include <ecto/ecto.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using ecto::tendrils;
namespace calib
{
  struct FiducialPoseFinder
  {
    typedef std::vector<cv::Point3f> object_pts_t;
    typedef std::vector<cv::Point2f> observation_pts_t;
    typedef FiducialPoseFinder C;

    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare(&C::in_points, "points", "Image points");
      in.declare(&C::in_ideal, "ideal", "The ideal object points.");
      in.declare(&C::in_K, "K", "The camera projection matrix.", cv::Mat::eye(3, 3, CV_32F));
      in.declare(&C::in_found, "found");
      out.declare(&C::out_R, "R", "3x3 Rotation matrix.");
      out.declare(&C::out_T, "T", "3x1 Translation vector.");
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      if (!*in_found)
      {
        out_T->release();
        out_R->release();
        return ecto::OK;
      }
      cv::Mat rvec, tvec;
      cv::solvePnP(*in_ideal, *in_points, *in_K, cv::Mat(), rvec, tvec, false);
      cv::Rodrigues(rvec, *out_R);
      *out_T = tvec;
      return ecto::OK;
    }
    ecto::spore<cv::Mat> out_R, out_T, in_K;
    ecto::spore<observation_pts_t> in_points;
    ecto::spore<object_pts_t> in_ideal;
    ecto::spore<bool> in_found;
  };
}
ECTO_CELL(calib, calib::FiducialPoseFinder, "FiducialPoseFinder",
          "Given 2D observations and their 3d corresponding points, estimate a 6 DOF pose.");
