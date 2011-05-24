#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using ecto::tendrils;

enum Pattern
{
  CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID
};

struct Camera
{
  cv::Mat K, D;
  cv::Size image_size;
};
static std::vector<cv::Point3f> calcChessboardCorners(cv::Size boardSize,
    float squareSize, Pattern patternType = CHESSBOARD)
{
  std::vector<cv::Point3f> corners;
  switch (patternType)
  {
  case CHESSBOARD:
  case CIRCLES_GRID:
    for (int i = 0; i < boardSize.height; i++)
      for (int j = 0; j < boardSize.width; j++)
        corners.push_back(
            cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
    break;

  case ASYMMETRIC_CIRCLES_GRID:
    for (int i = 0; i < boardSize.height; i++)
      for (int j = 0; j < boardSize.width; j++)
        corners.push_back(
            cv::Point3f(float((2 * j + i % 2) * squareSize),
                float(i * squareSize), 0));
    break;

  default:
    std::logic_error("Unknown pattern type.");
  }
  return corners;
}

struct PatternDetector: ecto::module_interface
{
  void configure(const tendrils& params, tendrils& in, tendrils& out)
  {
    in.declare<cv::Mat> ("input",
        "The grayscale image to search for a calibration pattern in.");
    out.declare<std::vector<cv::Point2f> > ("out",
        "The observed pattern points.");
    out.declare<bool> ("found", "Whether or not a pattern was found...");
    grid_size_ = cv::Size(params.get<int> ("cols"), params.get<int> ("rows"));
    choosePattern(params.get<std::string> ("pattern_type"));
  }

  void process(const tendrils& params, const tendrils& in, tendrils& out)
  {
    const cv::Mat& inm = in.get<cv::Mat> ("input");
    std::vector<cv::Point2f>& outv = out.get<std::vector<cv::Point2f> > ("out");
    switch (pattern_)
    {
    case ASYMMETRIC_CIRCLES_GRID:
      out.get<bool> ("found") = cv::findCirclesGrid(inm, grid_size_, outv,
          cv::CALIB_CB_ASYMMETRIC_GRID);
      break;
    case CHESSBOARD:
      out.get<bool> ("found")
          = cv::findChessboardCorners(inm, grid_size_, outv);
      break;
    case CIRCLES_GRID:
      out.get<bool> ("found") = cv::findCirclesGrid(inm, grid_size_, outv,
          cv::CALIB_CB_SYMMETRIC_GRID);
      break;
    }
  }

  void initialize(tendrils& params)
  {
    params.declare<int> ("rows", "Number of dots in row direction", 4);
    params.declare<int> ("cols", "Number of dots in col direction", 11);
    params.declare<std::string> ("pattern_type",
        "The pattern type, possible values are: [chessboard|circles|acircles]",
        "acircles");
  }

  void choosePattern(const std::string& pattern)
  {
    if (pattern == "chessboard")
    {
      pattern_ = CHESSBOARD;
    }
    else if (pattern == "circles")
    {
      pattern_ = CIRCLES_GRID;
    }
    else if (pattern == "acircles")
    {
      pattern_ = ASYMMETRIC_CIRCLES_GRID;
    }
    else
      throw std::runtime_error(
          "Unknown pattern type : " + pattern
              + " Please use: [chessboard|circles|acircles]");
  }
  cv::Size grid_size_;
  Pattern pattern_;
};

struct PatternDrawer: ecto::module_interface
{
  void configure(const tendrils& params, tendrils& in, tendrils& out)
  {
    SHOW();
    in.declare<cv::Mat> ("input", "The image to to find a vertical lazer line in.");
    in.declare<std::vector<cv::Point2f> > ("points", "Circle pattern points.");
    in.declare<bool> ("found", "Found the pattern");
    out.declare<cv::Mat> ("out", "Pattern Image");
    grid_size_ = cv::Size(params.get<int> ("cols"), params.get<int> ("rows"));
  }
  void process(const tendrils& params, const tendrils& in, tendrils& out)
  {
    SHOW();
    const cv::Mat& image = in.get<cv::Mat> ("input");
    const std::vector<cv::Point2f>& points =
        in.get<std::vector<cv::Point2f> > ("points");
    bool found = in.get<bool> ("found");
    cv::Mat& image_out = out.get<cv::Mat> ("out");
    image.copyTo(image_out);
    cv::drawChessboardCorners(image_out, grid_size_, points, found);
  }
  void initialize(tendrils& params)
  {
    SHOW();
    params.declare<int> ("rows", "Number of dots in row direction", 4);
    params.declare<int> ("cols", "Number of dots in col direction", 11);
  }
  cv::Size grid_size_;
};

struct CameraCalibrator: ecto::module_interface
{
  typedef std::vector<cv::Point3f> object_pts_t;
  typedef std::vector<cv::Point2f> observation_pts_t;
  void configure(const tendrils& params, tendrils& in, tendrils& out)
  {
    in.declare<observation_pts_t> ("points", "Circle pattern points.");
    in.declare<cv::Mat> ("image", "Image that is used for calibration");
    in.declare<bool> ("found", "Found the pattern");
    out.declare<float> ("norm",
        "Norm of the input points to all previous points observed.");
    grid_size_ = cv::Size(params.get<int> ("cols"), params.get<int> ("rows"));
    board_pts_
        = calcChessboardCorners(grid_size_, 1.0, ASYMMETRIC_CIRCLES_GRID);
    n_obs_ = params.get<int> ("n_obs");
    object_pts_.clear();
    norm_thresh_ = 150; //pixel values;
    calibrated_ = false;
  }
  double calcDistance(const observation_pts_t& in) const
  {
    cv::Mat p_in(in);
    double norm = 10e6;
    for (size_t i = 0; i < observation_pts_.size(); i++)
    {
      cv::Mat p_o(observation_pts_[i]);
      cv::Mat diff = p_in - p_o;
      norm = std::min(cv::norm(diff), norm);
    }
    return norm;
  }
  void process(const tendrils& params, const tendrils& in, tendrils& out)
  {
    const observation_pts_t& points_in = in.get<observation_pts_t> ("points");
    bool found = in.get<bool> ("found");
    float norm = 0;
    if (found)
    {
      norm = calcDistance(points_in);

      if (norm > norm_thresh_ || observation_pts_.empty())
      {
        std::cout << "distance: " << norm << std::endl << "capturing ..."
            << std::endl;
        object_pts_.push_back(board_pts_);
        observation_pts_.push_back(points_in);
      }

    }
    if (int(observation_pts_.size()) > n_obs_ && !calibrated_)
    {
      std::vector<cv::Mat> rvecs, tvecs;
      int flags = CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_FIX_PRINCIPAL_POINT;
      double rms = cv::calibrateCamera(object_pts_, observation_pts_,
          in.get<cv::Mat> ("image").size(), camera_.K, camera_.D, rvecs, tvecs,
          flags);
      std::cout << "K = " << camera_.K << std::endl;
      std::cout << "D = " << camera_.D << std::endl;

      printf("RMS error reported by calibrateCamera: %g\n", rms);
      calibrated_ = true;
    }

    out.get<float> ("norm") = norm;
  }
  void initialize(tendrils& params)
  {
    SHOW();
    params.declare<int> ("rows", "Number of dots in row direction", 4);
    params.declare<int> ("cols", "Number of dots in col direction", 11);
    params.declare<int> ("n_obs", "Number of observations", 50);
    params.declare<float> ("square_size", "Number of observations", 25);
  }
  cv::Size grid_size_;
  int n_obs_;
  float norm_thresh_;
  bool calibrated_;
  object_pts_t board_pts_;
  std::vector<object_pts_t> object_pts_;
  std::vector<observation_pts_t> observation_pts_;
  Camera camera_;
};

BOOST_PYTHON_MODULE(calib)
{
  ecto::wrap<PatternDetector>("PatternDetector");
  ecto::wrap<PatternDrawer>("PatternDrawer");
  ecto::wrap<CameraCalibrator>("CameraCalibrator");
}
