#include <limits.h>

#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ecto/registry.hpp>

#include <opencv2/rgbd/rgbd.hpp>

using ecto::tendrils;
namespace calib
{
  struct PlanarSegmentation
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<float>("x_crop", "The amount to keep in the x direction (meters) relative\n"
                            "to the coordinate frame defined by the pose.",
                            0.15);
      params.declare<float>("y_crop", "The amount to keep in the y direction (meters) relative to\n"
                            "the coordinate frame defined by the pose.",
                            0.15);
      params.declare<float>("z_crop", "The amount to keep in the z direction (meters) relative to\n"
                            "the coordinate frame defined by the pose.",
                            0.5);
      params.declare<float>("z_min", "The amount to crop above the plane, in meters.", 0.0075);
    }

    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare<cv::Mat>("depth", "The depth image to segment");
      //FIXME use a pose object here?
      in.declare(&PlanarSegmentation::R_, "R", "The pose rotation matrix.");
      in.declare(&PlanarSegmentation::T_, "T", "The pose translation vector.");
      in.declare(&PlanarSegmentation::K_, "K", "The camera matrix.");
      out.declare<cv::Mat>("mask", "The output mask, determined by the segmentation.\n"
                           "255 is the value for objects satisfying the constraints.\n"
                           "0 otherwise.");
    }
    void
    configure(const tendrils& p, const tendrils& inputs, const tendrils& outputs)
    {
      z_crop = p["z_crop"];
      x_crop = p["x_crop"];
      y_crop = p["y_crop"];
      z_min = p["z_min"];
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      cv::Mat depth = in.get<cv::Mat>("depth");
      if (depth.empty())
        return ecto::OK;

      if (R_->empty() || T_->empty() || K_->empty())
        return ecto::OK;

      cv::Matx33f R = *R_, K = *K_;
      cv::Vec3f T = *T_;

      cv::Mat mask = cv::Mat::zeros(depth.size(), CV_8UC1);

      box_mask.create(depth.size());
      box_mask.setTo(cv::Scalar(0));

      std::vector<cv::Point3f> box(8);
      box[0] = cv::Point3f(*x_crop, *y_crop, *z_min);
      box[1] = cv::Point3f(-*x_crop, *y_crop, *z_min);
      box[2] = cv::Point3f(-*x_crop, -*y_crop, *z_min);
      box[3] = cv::Point3f(*x_crop, -*y_crop, *z_min);
      box[4] = cv::Point3f(*x_crop, *y_crop, *z_crop);
      box[5] = cv::Point3f(-*x_crop, *y_crop, *z_crop);
      box[6] = cv::Point3f(-*x_crop, -*y_crop, *z_crop);
      box[7] = cv::Point3f(*x_crop, -*y_crop, *z_crop);

      std::vector<cv::Point2f> projected, hull;
      cv::Vec3f rvec;
      cv::Rodrigues(R, rvec);
      cv::projectPoints(box, rvec, T, K, cv::Mat(4, 1, CV_64FC1, cv::Scalar(0)), projected);

      cv::convexHull(projected, hull, true);
      std::vector<cv::Point> points(hull.size());
      std::copy(hull.begin(), hull.end(), points.begin());
      cv::fillConvexPoly(box_mask, points.data(), points.size(), cv::Scalar::all(255));

      std::vector<cv::Vec2i> mask_points(depth.size().area());
      size_t index = 0;
      for (int v = 0; v < depth.rows; ++v) {
        uchar* r = box_mask.ptr<uchar>(v, 0);

        for (int u = 0; u < depth.cols; ++u, ++r)
         if (*r)
           mask_points[index++] = cv::Vec2i(u, v);
      }
      mask_points.resize(index);

      if (index == 0)
        return ecto::OK;

      std::vector<cv::Vec3f> points3d;
      depthTo3dSparse(depth, K, mask_points, points3d);

      cv::Vec3f p_r, Tx(T); //Translation
      cv::Matx33f Rx = R.t(); //inverse Rotation
      std::vector<cv::Vec3f>::iterator point = points3d.begin(), end = points3d.end();
      std::vector<cv::Vec2i>::iterator mask_point = mask_points.begin();

      double z_min_ = *z_min, z_max_ = *z_crop;
      while (point != end)
      {
        //calculate the point based on the depth
        p_r = Rx * (*point - Tx);
        if (!cvIsNaN((*point)[0])) {
          if ((p_r(2) > z_min_
              && p_r(2) < z_max_
              && std::abs(p_r(0)) < *x_crop
              && std::abs(p_r(1)) < *y_crop) || true) {
            mask.at<uint8_t>((*mask_point)[1], (*mask_point)[0]) = 255;
          }
        }
        ++point;
        ++mask_point;
      }
      out["mask"] << mask;
      return ecto::OK;
    }
    ecto::spore<float> x_crop, y_crop, z_crop, z_min;
    cv::Mat_<uint8_t> box_mask;
    ecto::spore<cv::Mat> K_, R_, T_;
  }
  ;
}
ECTO_CELL(calib, calib::PlanarSegmentation, "PlanarSegmentation", "Given a pose, "
"assuming it describes the center of the object coordinate system and "
"lies on a plane, segment the object from the plane");
