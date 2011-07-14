#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/filesystem.hpp>
#include <algorithm>

namespace fs = boost::filesystem;
using ecto::tendrils;

enum Pattern
{
  CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID
};

static std::vector<cv::Point3f> calcChessboardCorners(
                                                      cv::Size boardSize,
                                                      float squareSize,
                                                      Pattern patternType = CHESSBOARD,
                                                      cv::Point3f offset = cv::Point3f())
{
  std::vector<cv::Point3f> corners;
  switch (patternType)
  {
    case CHESSBOARD:
    case CIRCLES_GRID:
      for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
          corners.push_back(
                            cv::Point3f(float(j * squareSize),
                                        float(i * squareSize), 0) + offset);
      break;
    case ASYMMETRIC_CIRCLES_GRID:
      for (int i = boardSize.height-1; i >= 0; i--)
        for (int j = 0; j < boardSize.width; j++)
          corners.push_back(
                            cv::Point3f(float(i * squareSize),
                                        float((2 * j + i % 2) * squareSize), 0) + offset);
      break;
    default:
      std::logic_error("Unknown pattern type.");
  }
  return corners;
}


struct PatternProjector
{
  typedef std::vector<cv::Point3f> pts3d_t;
  typedef std::vector<cv::Point2f> pts2d_t;

  static void declare_params(tendrils& params)
  {
    params.declare<int> ("rows", "Number of dots in row direction", 5);
    params.declare<int> ("cols", "Number of dots in col direction", 3);
    params.declare<int> ("image_width", "The width of image", 640);
    params.declare<int> ("image_height", "The height of image", 480);
  }

  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    out.declare<cv::Mat> ("pattern", "Pattern Image");
    out.declare<pts2d_t> ("points", "The pattern points in pixels.");
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    cv::Size grid_size_,image_size_;
    pts3d_t  points_;
    pts2d_t points_out_;
    float square_size_;
    grid_size_ = cv::Size(params.get<int> ("cols"), params.get<int> ("rows"));
    image_size_ = cv::Size(params.get<int> ("image_width"), params.get<int> ("image_height"));

    square_size_ = std::min(image_size_.width/float(grid_size_.width + 2), image_size_.height/float(grid_size_.height + 3));
    points_ = calcChessboardCorners(grid_size_, square_size_, ASYMMETRIC_CIRCLES_GRID,cv::Point3f(square_size_,square_size_,0));
    cv::Mat image_out = cv::Mat::zeros(image_size_,CV_8UC1);
    for(size_t i = 0; i < points_.size(); i++)
    {
      cv::Point3f p = points_[i];
      points_out_.push_back(cv::Point2f(p.x,p.y));
      cv::circle(image_out,cv::Point(p.x,p.y),square_size_ * 0.3,cv::Scalar::all(255),-1,8);
    }
    outputs["pattern"] << image_out;
    outputs["points"] << points_out_;
  }

};

ECTO_CELL(projector, PatternProjector,"PatternProjector", "Draws a dot pattern.");
