#include <ecto/ecto.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "calib.hpp"
#include <boost/format.hpp>

using ecto::tendrils;

static const char* POINTS = "points_%04d";
static const char* IDEAL = "ideal_%04d";
static const char* FOUND = "found_%04d";

namespace calib
{
  struct GatherPoints
  {
    typedef std::vector<cv::Point3f> object_pts_t;
    typedef std::vector<cv::Point2f> observation_pts_t;

    static void
    declare_params(tendrils& params)
    {
      params.declare<int>("N", "Number of patterns to gather", 2);
    }
    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      int N;
      params["N"] >> N;
      for (int i = 0; i < N; i++)
      {
        in.declare<observation_pts_t>(boost::str(boost::format(POINTS) % i), "Image points");
        in.declare<object_pts_t>(boost::str(boost::format(IDEAL) % i), "The ideal object points.");
        in.declare<bool>(boost::str(boost::format(FOUND) % i));
      }
      out.declare<observation_pts_t>("out", "The observed pattern points.");
      out.declare<object_pts_t>("ideal", "The ideal pattern points.");
      out.declare<bool>("found", "Found some points.");

    }
    void
    configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      params["N"] >> N;
    }
    int
    process(const tendrils& in, const tendrils& out)
    {
      object_pts_t obj_pts;
      observation_pts_t observe_pts;
      bool found_any = false;
      for (int i = 0; i < N; i++)
      {
        bool found;
        in[boost::str(boost::format(FOUND) % i)] >> found;
        if (!found)
          continue;
        found_any = true;
        object_pts_t ideal;
        observation_pts_t points;
        in[boost::str(boost::format(POINTS) % i)] >> points;
        in[boost::str(boost::format(IDEAL) % i)] >> ideal;
        obj_pts.insert(obj_pts.end(), ideal.begin(), ideal.end());
        observe_pts.insert(observe_pts.end(), points.begin(), points.end());
      }
      out["found"] << found_any;
      out["ideal"] << obj_pts;
      out["out"] << observe_pts;
      return ecto::OK;
    }
    int N;
  };
}
ECTO_CELL(calib,  calib::GatherPoints, "GatherPoints", "Gather points found by multiple patterns.");
