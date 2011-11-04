#include <limits.h>

#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ecto/registry.hpp>

#include "impl/depth_to_3d.h"

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
      in.declare<cv::Mat>("R", "The pose rotation matrix.");
      in.declare<cv::Mat>("T", "The pose translation vector.");
      in.declare<cv::Mat>("K", "The camera matrix.");
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

    void
    computeNormal(const cv::Mat& R, const cv::Mat& T, cv::Matx<double, 3, 1>& N, cv::Matx<double, 3, 1>& O)
    {

      cv::Vec3d z(0, 0, 1);
      //std::cout << cv::Mat(O) << std::endl;
      cv::Mat N_ = R * cv::Mat(z);
      cv::Mat O_ = T + N_ * (*z_min);
      O = O_;
      N = N_;
      //compute the offset from vector from the normal.
      //std::cout << "N = " << N << std::endl;
      //std::cout << "O = " << O << std::endl;

    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      cv::Mat depth = in.get<cv::Mat>("depth");
      if (depth.empty())
        return ecto::OK;

      cv::Mat R, T, K;
      in.get<cv::Mat>("R").convertTo(R, CV_64F);
      in.get<cv::Mat>("T").convertTo(T, CV_64F);
      in.get<cv::Mat>("K").convertTo(K, CV_64F);

      if (R.empty() || T.empty() || K.empty())
        return ecto::OK;

      cv::Mat mask = cv::Mat::zeros(depth.size(), CV_8UC1);

      box_mask.create(depth.size());
      box_mask.setTo(cv::Scalar(0));

      std::vector<cv::Point3f> box(8);
      box[0] = cv::Point3f(*x_crop, *y_crop, -*z_min);
      box[1] = cv::Point3f(-*x_crop, *y_crop, -*z_min);
      box[2] = cv::Point3f(-*x_crop, -*y_crop, -*z_min);
      box[3] = cv::Point3f(*x_crop, -*y_crop, -*z_min);
      box[4] = cv::Point3f(*x_crop, *y_crop, -*z_crop);
      box[5] = cv::Point3f(-*x_crop, *y_crop, -*z_crop);
      box[6] = cv::Point3f(-*x_crop, -*y_crop, -*z_crop);
      box[7] = cv::Point3f(*x_crop, -*y_crop, -*z_crop);

      std::vector<cv::Point2f> projected, hull;
      cv::Mat rvec;
      cv::Rodrigues(R, rvec);
      cv::projectPoints(box, R, -T, K, cv::Mat::zeros(1, 4, CV_64F), projected);
//      std::cout << "Rvec: " << rvec << std::endl;
//      std::cout << "T: " << T << std::endl;
//      std::cout << "K: " << K << std::endl;
//      std::cout << "Box: " << cv::Mat(box) << std::endl;
//      std::cout << "Projected: " << cv::Mat(projected) << std::endl;

      cv::convexHull(projected, hull, true);
      std::vector<cv::Point> points(hull.size());
      std::copy(hull.begin(), hull.end(), points.begin());
      cv::fillConvexPoly(box_mask, points.data(), points.size(), cv::Scalar::all(255));
//
//      out["mask"] << cv::Mat(box_mask);
//      return ecto::OK;
      int width = mask.size().width;
      int height = mask.size().height;

      cv::Mat_<cv::Vec3f> points3d;
      depthTo3dMask(K, depth, box_mask, points3d);
      if (points3d.empty())
        return ecto::OK;

      cv::Matx<double, 3, 1> p, p_r, Tx(T); //Translation
      cv::Matx<double, 3, 3> Rx; //inverse Rotation
      Rx = cv::Mat(R.t());
      cv::Mat_<cv::Vec3f>::iterator point = points3d.begin(), end = points3d.end();

      double z_min_ = *z_min, z_max_ = *z_crop, x_min_ = -*x_crop, x_max_ = *x_crop, y_min_ = -*y_crop,
          y_max_ = *y_crop;
      float fx = K.at<double>(0, 0);
      float cx = K.at<double>(0,2);
      float cy = K.at<double>(1,2);
      while (point != end)
      {
        //calculate the point based on the depth
        p(0) = (*point).val[0];
        p(1) = (*point).val[1];
        p(2) = (*point).val[2];
        ++point;
        p_r = Rx * (p - Tx);
        int u = p(0) * fx / p(2) + cx + 0.5;
        int v = p(1) * fx / p(2) + cy + 0.5;
        if (p_r(2) > z_min_ && p_r(2) < z_max_ && p_r(0) > x_min_ && p_r(0) < x_max_ && p_r(1) > y_min_
            && p_r(1) < y_max_)
          mask.at<uint8_t>(v, u) = 255;
      }
      out["mask"] << mask;
      return ecto::OK;
    }
    ecto::spore<float> x_crop, y_crop, z_crop, z_min;
    cv::Mat_<uint8_t> box_mask;

  }
  ;
}
ECTO_CELL(calib, calib::PlanarSegmentation, "PlanarSegmentation", "Given a pose, "
"assuming it describes the center of the object coordinate system and "
"lies on a plane, segment the object from the plane");
