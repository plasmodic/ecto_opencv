#include <ecto/ecto.hpp>
#include <boost/python.hpp>

#include <opencv2/rgbd/rgbd.hpp>

namespace bp = boost::python;

ECTO_DEFINE_MODULE(rgbd)
{
  bp::enum_<cv::RgbdNormals::RGBD_NORMALS_METHOD>("RgbdNormalsTypes").value("SRI",
                                                                            cv::RgbdNormals::RGBD_NORMALS_METHOD_SRI).value(
      "FALS", cv::RgbdNormals::RGBD_NORMALS_METHOD_FALS).value(
      "LINEMOD", cv::RgbdNormals::RGBD_NORMALS_METHOD_LINEMOD);
}
