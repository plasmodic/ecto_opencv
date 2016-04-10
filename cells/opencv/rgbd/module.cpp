#include <ecto/ecto.hpp>
#include <boost/python.hpp>

#include <opencv2/core/core.hpp>
#if CV_MAJOR_VERSION == 3
#include <opencv2/rgbd.hpp>
using cv::rgbd::RgbdNormals;
using cv::rgbd::DepthCleaner;
#else
#include <opencv2/rgbd/rgbd.hpp>
using cv::RgbdNormals;
using cv::DepthCleaner;
#endif

namespace bp = boost::python;

ECTO_DEFINE_MODULE(rgbd)
{
  bp::enum_<RgbdNormals::RGBD_NORMALS_METHOD>("RgbdNormalsTypes").value("SRI",
                                                                            RgbdNormals::RGBD_NORMALS_METHOD_SRI).value(
      "FALS", RgbdNormals::RGBD_NORMALS_METHOD_FALS).value(
      "LINEMOD", RgbdNormals::RGBD_NORMALS_METHOD_LINEMOD);
  bp::enum_<DepthCleaner::DEPTH_CLEANER_METHOD>("DepthCleanerTypes").value("NIL",
                                                                            DepthCleaner::DEPTH_CLEANER_NIL);
}
