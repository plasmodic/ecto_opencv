#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>

#include <opencv2/calib3d/calib3d.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;
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

void readOpenCVCalibration(Camera& camera, const std::string& calibfile)
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

void writeOpenCVCalibration(const Camera& camera, const std::string& calibfile)
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

static std::vector<cv::Point3f> calcChessboardCorners(cv::Size boardSize, float squareSize,
                                                      Pattern patternType = CHESSBOARD)
{
  std::vector<cv::Point3f> corners;
  switch (patternType)
    {
    case CHESSBOARD:
    case CIRCLES_GRID:
      for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
          corners.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
      break;
    case ASYMMETRIC_CIRCLES_GRID:
      for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
          corners.push_back(cv::Point3f(float(i * squareSize),float((2 * j + i % 2) * squareSize), 0));
      break;
    default:
      std::logic_error("Unknown pattern type.");
    }
  return corners;
}

struct PatternDetector
{
  typedef std::vector<cv::Point3f> object_pts_t;

  static void declare_params(tendrils& params)
  {
    params.declare<int> ("rows", "Number of dots in row direction", 4);
    params.declare<int> ("cols", "Number of dots in col direction", 11);
    params.declare<float> ("square_size", "The dimensions of each square", 1.0f);
    params.declare<std::string> ("pattern_type",
                                 "The pattern type, possible values are: [chessboard|circles|acircles]", "acircles");

  }

  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    in.declare<cv::Mat> ("input", "The grayscale image to search for a calibration pattern in.");
    out.declare<std::vector<cv::Point2f> > ("out", "The observed pattern points.");
    out.declare<object_pts_t> ("ideal", "The ideal pattern points.");
    out.declare<bool> ("found", "Whether or not a pattern was found...");
  }

  void configure(tendrils& params)
  {
    grid_size_ = cv::Size(params.get<int> ("cols"), params.get<int> ("rows"));
    choosePattern(params.get<std::string> ("pattern_type"));
    square_size_ = params.get<float> ("square_size");
    ideal_pts_ = calcChessboardCorners(grid_size_, square_size_, pattern_);
  }

  int process(const tendrils& in, tendrils& out)
  {
    const cv::Mat& inm = in.get<cv::Mat> ("input");
    std::vector<cv::Point2f>& outv = out.get<std::vector<cv::Point2f> > ("out");
    switch (pattern_)
      {
      case ASYMMETRIC_CIRCLES_GRID:
        out.get<bool> ("found") = cv::findCirclesGrid(inm, grid_size_, outv, cv::CALIB_CB_ASYMMETRIC_GRID);
        break;
      case CHESSBOARD:
        out.get<bool> ("found") = cv::findChessboardCorners(inm, grid_size_, outv);
        break;
      case CIRCLES_GRID:
        out.get<bool> ("found") = cv::findCirclesGrid(inm, grid_size_, outv, cv::CALIB_CB_SYMMETRIC_GRID);
        break;
      }
    out.get<object_pts_t> ("ideal") = ideal_pts_;
    return 0;
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
      throw std::runtime_error("Unknown pattern type : " + pattern + " Please use: [chessboard|circles|acircles]");
  }
  cv::Size grid_size_;
  float square_size_;
  Pattern pattern_;
  object_pts_t ideal_pts_;

};

struct PatternDrawer
{

  static void declare_params(tendrils& params)
  {
    params.declare<int> ("rows", "Number of dots in row direction", 4);
    params.declare<int> ("cols", "Number of dots in col direction", 11);
  }

  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    in.declare<cv::Mat> ("input", "The image to to find a vertical lazer line in.");
    in.declare<std::vector<cv::Point2f> > ("points", "Circle pattern points.");
    in.declare<bool> ("found", "Found the pattern");
    out.declare<cv::Mat> ("out", "Pattern Image");
  }

  void configure(const tendrils& params)
  {
    grid_size_ = cv::Size(params.get<int> ("cols"), params.get<int> ("rows"));
  }

  int process(const tendrils& in, tendrils& out)
  {
    const cv::Mat& image = in.get<cv::Mat> ("input");
    const std::vector<cv::Point2f>& points = in.get<std::vector<cv::Point2f> > ("points");
    bool found = in.get<bool> ("found");
    cv::Mat& image_out = out.get<cv::Mat> ("out");
    image.copyTo(image_out);
    cv::drawChessboardCorners(image_out, grid_size_, points, found);
    return 0;
  }
  cv::Size grid_size_;
};

struct CameraCalibrator
{
  typedef std::vector<cv::Point3f> object_pts_t;
  typedef std::vector<cv::Point2f> observation_pts_t;

  static void declare_params(tendrils& params)
  {
    params.declare<std::string> ("output_file_name", "The name of the camera calibration file", "camera.yml");
    params.declare<int> ("n_obs", "Number of observations", 50);
  }

  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    in.declare<observation_pts_t> ("points", "Image points");
    in.declare<object_pts_t> ("ideal", "The ideal object points.");
    in.declare<bool> ("found");
    in.declare<cv::Mat> ("image", "The image that is being used for calibration");
    out.declare<float> ("norm", "Norm of the input points to all previous points observed.");
    out.declare<bool> ("calibrated", "Done calibration", false);
  }

  void configure(const tendrils& params)
  {
    n_obs_ = params.get<int> ("n_obs");
    camera_output_file_ = params.get<std::string> ("output_file_name");
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
  int process(const tendrils& in, tendrils& out)
  {
    const observation_pts_t& points_in = in.get<observation_pts_t> ("points");
    const object_pts_t& board_pts = in.get<object_pts_t> ("ideal");
    bool found = in.get<bool> ("found");
    float norm = 0;
    if (found)
      {
        norm = calcDistance(points_in);

        if (norm > norm_thresh_ || observation_pts_.empty())
          {
            std::cout << "distance: " << norm << std::endl << "capturing ..." << std::endl;
            object_pts_.push_back(board_pts);
            observation_pts_.push_back(points_in);
          }

      }
    if (int(observation_pts_.size()) > n_obs_ && !calibrated_)
      {
        std::cout << "Calibrating the camera, given " << n_obs_ << " observations" << std::endl;
        std::vector<cv::Mat> rvecs, tvecs;
        int flags = CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_FIX_PRINCIPAL_POINT;
        camera_.image_size = in.get<cv::Mat> ("image").size();
        double rms = cv::calibrateCamera(object_pts_, observation_pts_, camera_.image_size, camera_.K, camera_.D,
                                         rvecs, tvecs, flags);
        std::cout << "K = " << camera_.K << std::endl;
        std::cout << "D = " << camera_.D << std::endl;

        std::cout << "camera size = (" << camera_.image_size.width << ", " << camera_.image_size.height << ")"
            << std::endl;

        writeOpenCVCalibration(camera_, camera_output_file_);

        printf("RMS error reported by calibrateCamera: %g\n", rms);
        calibrated_ = true;
      }

    out.get<float> ("norm") = norm;
    out.get<bool> ("calibrated") = calibrated_;
    return 0;
  }
  cv::Size grid_size_;
  int n_obs_;
  float norm_thresh_;
  bool calibrated_;
  std::vector<object_pts_t> object_pts_;
  std::vector<observation_pts_t> observation_pts_;
  Camera camera_;
  std::string camera_output_file_;
};

struct CameraIntrinsics
{
  static void declare_params(tendrils& params)
  {
    params.declare<std::string> ("camera_file", "The camera calibration file. Typically a .yml", "camera.yml");
  }
  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    out.declare<cv::Size> ("image_size", "The image size.");
    out.declare<cv::Mat> ("K", "3x3 camera intrinsic matrix.");
    out.declare<std::string> ("camera_model", "The camera model. e.g pinhole,...", "pinhole");
  }
  void configure(tendrils& params)
  {
    readOpenCVCalibration(camera, params.get<std::string> ("camera_file"));
  }
  int process(const tendrils& in, tendrils& out)
  {
    out.get<cv::Mat>("K") = camera.K;
    out.get<cv::Size>("image_size") = camera.image_size;
    return 0;
  }
  Camera camera;
};

struct FiducialPoseFinder
{
  typedef std::vector<cv::Point3f> object_pts_t;
  typedef std::vector<cv::Point2f> observation_pts_t;

  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    in.declare<observation_pts_t> ("points", "Image points");
    in.declare<object_pts_t> ("ideal", "The ideal object points.");
    in.declare<cv::Mat> ("K", "The camera projection matrix.", cv::Mat::eye(3, 3, CV_32F));
    in.declare<bool> ("found");

    out.declare<cv::Mat> ("R", "3x3 Rotation matrix.");
    out.declare<cv::Mat> ("T", "3x1 Translation vector.");
  }

  int process(const tendrils& in, tendrils& out)
  {
    if (!in.get<bool> ("found"))
      return 0;
    const observation_pts_t& observation_points = in.get<observation_pts_t> ("points");
    const object_pts_t& object_points = in.get<object_pts_t> ("ideal");
    cv::Mat K = in.get<cv::Mat> ("K");
    cv::Mat rvec, tvec;
    cv::solvePnP(object_points, observation_points, K, cv::Mat(), rvec, tvec, false);
    cv::Rodrigues(rvec, out.get<cv::Mat> ("R"));
    out.get<cv::Mat> ("T") = tvec;
    //std::cout << out.get<cv::Mat> ("R") << std::endl << out.get<cv::Mat> ("T") << std::endl;
    return 0;
  }
};

struct PoseDrawer
{
  static void draw(cv::Mat& drawImage, const cv::Mat& K, const cv::Mat& R, const cv::Mat T)
  {
    using namespace cv;

    if (R.empty() || T.empty())
      return;
    Point3f z(0, 0, 0.25);
    Point3f x(0.25, 0, 0);
    Point3f y(0, 0.25, 0);
    Point3f o(0, 0, 0);
    vector<Point3f> op(4);
    op[1] = x, op[2] = y, op[3] = z, op[0] = o;
    vector<Point2f> ip;
    projectPoints(Mat(op), R, T, K, Mat(4, 1, CV_64FC1, Scalar(0)), ip);

    vector<Scalar> c(4); //colors
    c[0] = Scalar(255, 255, 255);
    c[1] = Scalar(255, 0, 0);//x
    c[2] = Scalar(0, 255, 0);//y
    c[3] = Scalar(0, 0, 255);//z
    line(drawImage, ip[0], ip[1], c[1]);
    line(drawImage, ip[0], ip[2], c[2]);
    line(drawImage, ip[0], ip[3], c[3]);
    string scaleText = "scale 0.25 meters";
    int baseline = 0;
    Size sz = getTextSize(scaleText, CV_FONT_HERSHEY_SIMPLEX, 1, 1, &baseline);
    rectangle(drawImage, Point(10, 30 + 5), Point(10, 30) + Point(sz.width, -sz.height - 5), Scalar::all(0), -1);
    putText(drawImage, scaleText, Point(10, 30), CV_FONT_HERSHEY_SIMPLEX, 1.0, c[0], 1, CV_AA, false);
    putText(drawImage, "Z", ip[3], CV_FONT_HERSHEY_SIMPLEX, 1.0, c[3], 1, CV_AA, false);
    putText(drawImage, "Y", ip[2], CV_FONT_HERSHEY_SIMPLEX, 1.0, c[2], 1, CV_AA, false);
    putText(drawImage, "X", ip[1], CV_FONT_HERSHEY_SIMPLEX, 1.0, c[1], 1, CV_AA, false);

  }
  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    in.declare<cv::Mat> ("K", "The camera projection matrix.");
    in.declare<cv::Mat> ("R", "3x3 Rotation matrix.");
    in.declare<cv::Mat> ("T", "3x1 Translation vector.");
    in.declare<cv::Mat> ("image", "The original image to draw the pose onto.");
    out.declare<cv::Mat> ("output", "The pose of the fiducial, drawn on an image");
  }

  int process(const tendrils& in, tendrils& out)
  {
    cv::Mat K, R, T, image;
    K = in.get<cv::Mat> ("K");
    R = in.get<cv::Mat> ("R");
    T = in.get<cv::Mat> ("T");
    image = in.get<cv::Mat> ("image");
    cv::Mat& output = out.get<cv::Mat> ("output");
    image.copyTo(output);
    draw(output, K, R, T);
    return 0;
  }
};

BOOST_PYTHON_MODULE(calib)
{
  ecto::wrap<PatternDetector>("PatternDetector");
  ecto::wrap<PatternDrawer>("PatternDrawer");
  ecto::wrap<CameraCalibrator>("CameraCalibrator", "Accumulates observed points and ideal 3d points, and runs "
    "opencv calibration routines after some number of "
    "satisfactorily unique observations.");
  ecto::wrap<CameraIntrinsics>("CameraIntrinsics",
                               "This reads a camera calibration file and puts the results on the outputs.");
  ecto::wrap<FiducialPoseFinder>("FiducialPoseFinder");
  ecto::wrap<PoseDrawer>("PoseDrawer");

}
