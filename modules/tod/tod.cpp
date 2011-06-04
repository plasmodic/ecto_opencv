#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
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
    params.declare<float> ("x_crop",
                           "The amount to keep in the x direction (meters) relative\n"
                             "to the coordinate frame defined by the pose.",
                           0.1);
    params.declare<float> ("y_crop",
                           "The amount to keep in the y direction (meters) relative to\n"
                             "the coordinate frame defined by the pose.", 0.1);
    params.declare<float> ("z_crop",
                           "The amount to keep in the z direction (meters) relative to\n"
                             "the coordinate frame defined by the pose.", 0.25);
  }

  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    in.declare<cv::Mat> ("depth", "The depth image to segment");
    //FIXME use a pose object here?
    in.declare<cv::Mat> ("R", "The pose rotation matrix.");
    in.declare<cv::Mat> ("T", "The pose translation vector.");
    in.declare<cv::Mat> ("K", "The camera matrix.");

    out.declare<cv::Mat> ("mask",
                          "The output mask, determined by the segmentation.\n"
                            "255 is the value for objects satisfying the constraints.\n"
                            "0 otherwise.");
  }
  void configure(tendrils& p)
  {
    z_crop = p.get<float> ("z_crop");
    x_crop = p.get<float> ("x_crop");
    y_crop = p.get<float> ("y_crop");

  }

  void computeNormal(const cv::Mat& R, const cv::Mat& T, cv::Mat& N, cv::Mat& O)
  {

    cv::Vec3d z(0, 0, 1);
    O = T;
    std::cout << "O = " << O << std::endl;
    //std::cout << cv::Mat(O) << std::endl;
    N = R * cv::Mat(z);
    std::cout << "N = " << N << std::endl;
    //std::cout << cv::Mat(N) << std::endl;

  }

  int process(const tendrils& in, tendrils& out)
  {
    cv::Mat R, T, K, depth;
    in.get<cv::Mat> ("R").convertTo(R, CV_64F);
    in.get<cv::Mat> ("T").convertTo(T, CV_64F);
    in.get<cv::Mat> ("K").convertTo(K, CV_64F);
    depth = in.get<cv::Mat> ("depth");

    if (R.empty() || T.empty() || K.empty() || depth.empty())
      return 0;
    cv::Mat& mask = out.get<cv::Mat> ("mask");

#if 1
    cv::Mat A_x;
    mask.create(depth.size(), CV_16UC1);
    mask = cv::Scalar(0);
    A_x = K.inv();
    cv::Mat N, O;
    computeNormal(R, T, N, O);

    cv::Mat numerator = (O).t() * N;

    int width = mask.size().width;
    int height = mask.size().height;
    cv::Mat_<uint16_t>::iterator it = mask.begin<uint16_t> ();
    std::vector<cv::Point3f> origin(1);
    origin[0] = cv::Point3f(0, 0, 0);
    std::vector<cv::Point2f> projected;
    cv::projectPoints(origin, R, T, K, cv::Mat(4, 1, CV_64FC1, cv::Scalar(0)),
                      projected);
    std::cout << cv::Mat(projected) << std::endl;
    for (int v = 0; v < height; v++)
      {
        for (int u = 0; u < width; u++, ++it)
          {
            cv::Mat xy1(cv::Vec3d(u, v, 1));
            cv::Mat A = A_x * xy1;
            cv::Mat k = numerator / ((A).t() * N);
            cv::Mat X = k.at<double> (0) * (A);
            if (u == int(projected[0].x) && v == int(projected[0].y))
              {
                std::cout << "u = " << u << " v = " << v << std::endl;
                std::cout << "zed = " << X.at<cv::Vec3d> (0)[2] * 1000 << "\n";
                std::cout << "depth = " << depth.at<uint16_t> (v, u) << "\n";
              }
            *it = uint16_t(X.at<cv::Vec3d> (0)[2] * 1000);
          }
      }
    mask = depth < (mask);

#else
    mask.create(depth.size(), CV_8UC1);
    mask = cv::Scalar(0);

    std::vector<cv::Point3f> box(8);
    box[0] = cv::Point3f(x_crop, y_crop, 0);
    box[1] = cv::Point3f(-x_crop, y_crop, 0);
    box[2] = cv::Point3f(-x_crop, -y_crop, 0);
    box[3] = cv::Point3f(x_crop, -y_crop, 0);
    box[4] = cv::Point3f(x_crop, y_crop, z_crop);
    box[5] = cv::Point3f(-x_crop, y_crop, z_crop);
    box[6] = cv::Point3f(-x_crop, -y_crop, z_crop);
    box[7] = cv::Point3f(x_crop, -y_crop, z_crop);

    std::vector<cv::Point2f> projected, hull;
    cv::projectPoints(box, R, T, K, cv::Mat(4, 1, CV_64FC1, cv::Scalar(0)),
        projected);

    cv::convexHull(projected, hull, true);
    std::vector<cv::Point> points(hull.size());
    std::copy(hull.begin(), hull.end(), points.begin());
    cv::fillConvexPoly(mask, points.data(), points.size(), cv::Scalar::all(255));
#endif
    return 0;
  }
  float x_crop, y_crop, z_crop;
};

}

BOOST_PYTHON_MODULE(tod)
{
  ecto::wrap<tod::PlanarSegmentation>("PlanarSegmentation", "Given a pose, "
    "assuming it describes the center of the object coordinate system and "
    "lies on a plane, segment the object from the plane");
}
