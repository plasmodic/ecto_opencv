#include "calib.hpp"

#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using ecto::tendrils;
using namespace calib;
namespace calib
{
  struct PatternDetector
  {
    typedef std::vector<cv::Point3f> object_pts_t;

    static std::vector<cv::Point3f>
    calcChessboardCorners(cv::Size boardSize, float squareSize, Pattern patternType = CHESSBOARD, cv::Point3f offset =
        cv::Point3f())
    {
      std::vector<cv::Point3f> corners;
      switch (patternType)
      {
        case CHESSBOARD:
          for (int i = boardSize.height -1; i >=0; i--)
            for (int j = 0; j < boardSize.width; j++)
              corners.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0) + offset);
          break;
        case CIRCLES_GRID:
          for (int i = 0; i < boardSize.height; i++)
            for (int j = 0; j < boardSize.width; j++)
              corners.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0) + offset);
          break;
        case ASYMMETRIC_CIRCLES_GRID:
          for (int i = 0; i < boardSize.height; i++)
            for (int j = 0; j < boardSize.width; j++)
              corners.push_back(cv::Point3f(float(i * squareSize), float((2 * j + i % 2) * squareSize), 0) + offset);
          break;
        default:
          throw std::logic_error("Unknown pattern type.");
          break;
      }
      return corners;
    }

    static void
    declare_params(tendrils& params)
    {
      params.declare<int>("rows", "Number of dots in row direction", 4);
      params.declare<int>("cols", "Number of dots in col direction", 11);
      params.declare<float>("square_size", "The dimensions of each square", 1.0f);
      params.declare<Pattern>("pattern_type", "The pattern type", ASYMMETRIC_CIRCLES_GRID);
      params.declare<float>("offset_x", "Offset in x.", 0);
      params.declare<float>("offset_y", "Offset in y.", 0);
      params.declare<float>("offset_z", "Offset in z.", 0);
    }

    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare<cv::Mat>("input", "The grayscale image to search for a calibration pattern in.");
      out.declare<std::vector<cv::Point2f> >("out", "The observed pattern points.");
      out.declare<object_pts_t>("ideal", "The ideal pattern points.");
      out.declare<bool>("found", "Whether or not a pattern was found...");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      grid_size_ = cv::Size(params.get<int>("cols"), params.get<int>("rows"));
      pattern_ = params.get<Pattern>("pattern_type");
      square_size_ = params.get<float>("square_size");
      cv::Point3f offset;
      params["offset_x"] >> offset.x;
      params["offset_y"] >> offset.y;
      params["offset_z"] >> offset.z;
      ideal_pts_ = calcChessboardCorners(grid_size_, square_size_, pattern_, offset);

    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      const cv::Mat& inm = in.get<cv::Mat>("input");
      std::vector<cv::Point2f>& outv = out.get<std::vector<cv::Point2f> >("out");
      if (inm.empty())
        return ecto::OK;
      switch (pattern_)
      {
        case ASYMMETRIC_CIRCLES_GRID:
          out.get<bool>("found") = cv::findCirclesGrid(inm, grid_size_, outv,
                                                       cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
          break;
        case CHESSBOARD:
          out.get<bool>("found") = cv::findChessboardCorners(inm, grid_size_, outv);
          break;
        case CIRCLES_GRID:
          out.get<bool>("found") = cv::findCirclesGrid(inm, grid_size_, outv, cv::CALIB_CB_SYMMETRIC_GRID);
          break;
      }
      out.get<object_pts_t>("ideal") = ideal_pts_;
      return 0;
    }

    cv::Size grid_size_;
    float square_size_;
    Pattern pattern_;
    object_pts_t ideal_pts_;
  };
}
ECTO_CELL(calib, PatternDetector, "PatternDetector", "Detect chessboards, circles, acircles");

