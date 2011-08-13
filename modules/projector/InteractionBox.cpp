#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/filesystem.hpp>
#include <algorithm>

namespace fs = boost::filesystem;
using ecto::tendrils;

struct InteractionBox
{
  typedef std::vector<cv::Point3f> pts3d_t;
  typedef std::vector<cv::Point2f> pts2d_t;

  static void
  declare_params(tendrils& params)
  {
    params.declare<float>("box_width", "Box width in meters", 0.10);
    params.declare<float>("box_height", "Box height in meters", 0.10);
    params.declare<float>("box_depth", "Box width in meters", 0.10);
    params.declare<float>("origin_x", "Origin in x", 0);
    params.declare<float>("origin_y", "Origin in y", 0);
    params.declare<float>("origin_z", "Origin in z", 0);
  }

  static void
  declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    in.declare<cv::Mat>("R", "rotation").required(true);
    ;
    in.declare<cv::Mat>("T", "translation").required(true);
    ;
    in.declare<cv::Mat>("points3d", "The 3d points.").required(true);
  }

  void
  configure(const tendrils& params, const tendrils& in, const tendrils& out)
  {
    box_width = params["box_width"];
    box_height = params["box_height"];
    box_depth = params["box_depth"];
    origin_x = params["origin_x"];
    origin_y = params["origin_y"];
    origin_z = params["origin_z"];
    R = in["R"];
    T = in["T"];
    points3d = in["points3d"];
  }

  int
  process(const tendrils&, const tendrils&)
  {
    std::vector<cv::Point3f> bounds(2);
    cv::Point3f origin(*origin_x, *origin_y, *origin_z);
    cv::Point3f t(T->at<float>(0), T->at<float>(1), T->at<float>(2));
    cv::Point3f maxp = t + origin + cv::Point3f(*box_width / 2, *box_height / 2, *box_depth / 2);
    cv::Point3f minp = t + origin + cv::Point3f(-*box_width / 2, -*box_height / 2, -*box_depth / 2);
    cv::Mat_<float> points = *points3d;
    cv::Mat_<float>::const_iterator begin = points.begin();
    cv::Mat_<float>::const_iterator end = points.end();
    int count = 0;
    while (begin != end)
    {
      cv::Point3f p;
      p.x = *(begin++);
      p.y = *(begin++);
      p.z = *(begin++);
      if (minp.x < p.x && p.x < maxp.x)
        if (minp.y < p.y && p.y < maxp.y)
          if (minp.z < p.z && p.z < maxp.z)

          {
            count++;
          }
    }
    if (count > 1)
      std::cout << count << std::endl;
    return ecto::OK;
  }
  ecto::spore<float> box_width, box_height, box_depth, origin_x, origin_y, origin_z;
  ecto::spore<cv::Mat> R, T, points3d;

};

ECTO_CELL(projector, InteractionBox, "InteractionBox", "Boxes that detect interaction.");
