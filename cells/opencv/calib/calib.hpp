#pragma once
#include <string>
#include <opencv2/core/core.hpp>
namespace calib
{
  enum Pattern
  {
    CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID
  };

  struct Camera
  {
    cv::Mat K, D;
    cv::Size image_size;
  };

  void
  readOpenCVCalibration(Camera& camera, const std::string& calibfile);
  void
  writeOpenCVCalibration(const Camera& camera, const std::string& calibfile);
}
