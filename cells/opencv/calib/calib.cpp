#include <ecto/ecto.hpp>
#include <boost/format.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/filesystem.hpp>

#include "calib.hpp"

namespace fs = boost::filesystem;
using ecto::tendrils;

namespace calib
{
  void
  readOpenCVCalibration(Camera& camera, const std::string& calibfile)
  {
    cv::FileStorage fs(calibfile, cv::FileStorage::READ);
    if (!fs.isOpened())
      throw std::runtime_error("Could not open " + calibfile + " for read.");
    cv::read(fs["camera_matrix"], camera.K, cv::Mat());
    cv::read(fs["distortion_coefficients"], camera.D, cv::Mat());
    cv::read(fs["image_width"], camera.image_size.width, 0);
    cv::read(fs["image_height"], camera.image_size.height, 0);
    if (camera.K.empty())
      throw std::runtime_error("The camera_matrix could not be read from the file");
    if (camera.K.size() != cv::Size(3, 3))
      throw std::runtime_error("The camera_matrix must be a 3x3 matrix");
  }

  void
  writeOpenCVCalibration(const Camera& camera, const std::string& calibfile)
  {
    cv::FileStorage fs(calibfile, cv::FileStorage::WRITE);
    if (!fs.isOpened())
      throw std::runtime_error("Could not open " + calibfile + " for write.");
    cvWriteComment(*fs, "camera intrinsics", 0);
    fs << "camera_matrix" << camera.K;
    fs << "distortion_coefficients" << camera.D;
    fs << "image_width" << camera.image_size.width;
    fs << "image_height" << camera.image_size.height;
  }
}
using namespace calib;
