#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ecto/registry.hpp>
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
                            0.1);
      params.declare<float>("y_crop", "The amount to keep in the y direction (meters) relative to\n"
                            "the coordinate frame defined by the pose.",
                            0.1);
      params.declare<float>("z_crop", "The amount to keep in the z direction (meters) relative to\n"
                            "the coordinate frame defined by the pose.",
                            0.25);
      params.declare<float>("z_min", "The amount to crop above the plane, in meters.", 0.02);
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
      cv::Mat R, T, K, depth;
      in.get<cv::Mat>("R").convertTo(R, CV_64F);

      in.get<cv::Mat>("T").convertTo(T, CV_64F);
      in.get<cv::Mat>("K").convertTo(K, CV_64F);
 
      cv::Mat mask;
      depth = in.get<cv::Mat>("depth");
      if (!depth.empty())
      {
        mask = cv::Mat::zeros(depth.size(), CV_8UC1);
      }
      else
        return ecto::OK;

      if (R.empty() || T.empty() || K.empty())
        return 0;
      
      if(depth.depth() == CV_16U)
      {
        cv::Mat temp;
        depth.convertTo(temp,CV_32F, 1.0/1000);
        depth = temp;
      }
      double fx, fy, cx, cy;
      fx = K.at<double>(0, 0);
      fy = K.at<double>(1, 1);
      cx = K.at<double>(0, 2);
      cy = K.at<double>(1, 2);

      box_mask.create(depth.size());
      box_mask.setTo(cv::Scalar(0));

      std::vector<cv::Point3f> box(8), tbox(8);
      box[0] = cv::Point3f(*x_crop, *y_crop, *z_min);
      box[1] = cv::Point3f(-*x_crop, *y_crop, *z_min);
      box[2] = cv::Point3f(-*x_crop, -*y_crop, *z_min);
      box[3] = cv::Point3f(*x_crop, -*y_crop, *z_min);
      box[4] = cv::Point3f(*x_crop, *y_crop, *z_crop);
      box[5] = cv::Point3f(-*x_crop, *y_crop, *z_crop);
      box[6] = cv::Point3f(-*x_crop, -*y_crop, *z_crop);
      box[7] = cv::Point3f(*x_crop, -*y_crop, *z_crop);

      cv::Mat RT(3, 4, CV_64F);
      cv::Mat _T(RT.colRange(3, 4));
      cv::Mat _R(RT.colRange(0, 3));
      R.copyTo(_R);
      T.copyTo(_T);
      cv::transform(box, tbox, RT);

      std::vector<cv::Point2f> projected, hull;
      cv::projectPoints(box, R, T, K, cv::Mat(4, 1, CV_64FC1, cv::Scalar(0)), projected);

      cv::convexHull(projected, hull, true);
      std::vector<cv::Point> points(hull.size());
      std::copy(hull.begin(), hull.end(), points.begin());
      cv::fillConvexPoly(box_mask, points.data(), points.size(), cv::Scalar::all(255));
      cv::Matx<double, 3, 3> A_x;

      int width = mask.size().width;
      int height = mask.size().height;
      cv::Mat_<float_t>::iterator dit = depth.begin<float_t>();
      cv::Mat_<uint8_t>::iterator it = mask.begin<uint8_t>();
      cv::Mat_<uint8_t>::iterator mit = box_mask.begin();

      cv::Matx<double, 3, 1> p, p_r, Tx(T); //Translation
      cv::Matx<double, 3, 3> Rx; //inverse Rotation
      Rx = cv::Mat(R.t());
//      std::cout << cv::Mat(Rx) << "\n" << cv::Mat(Tx) << std::endl;
//      std::cout << fx << " " << fy << " " << cx << " " << cy << " " << std::endl;
      double z_min_ = *z_min, z_max_ = *z_crop, x_min_ = -*x_crop, x_max_ = *x_crop, y_min_ = -*y_crop,
          y_max_ = *y_crop;
      for (int v = 0; v < height; v++)
      {
        for (int u = 0; u < width; u++, ++it, ++mit, ++dit)
        {
          if (*mit == 0)
            continue;
          //calculate the point based on the depth
          p(2) = *dit; //depth value in meters
          p(0) = (u - cx) * p(2) / fx;
          p(1) = (v - cy) * p(2) / fy;
          p_r = Rx * (p - Tx);
//          std::cout <<"p=" << cv::Mat(p) << ",p_r="<< cv::Mat(p_r) << std::endl;
          if (p_r(2) > z_min_ && p_r(2) < z_max_ && p_r(0) > x_min_ && p_r(0) < x_max_ && p_r(1) > y_min_
              && p_r(1) < y_max_)
            *it = 255;
        }
      }
      out["mask"] << mask;
      return ecto::OK;
    }
    ecto::spore<float> x_crop, y_crop, z_crop, z_min;
    cv::Mat_<uint8_t> box_mask;

  };
}
ECTO_CELL(calib, calib::PlanarSegmentation, "PlanarSegmentation", "Given a pose, "
"assuming it describes the center of the object coordinate system and "
"lies on a plane, segment the object from the plane");
