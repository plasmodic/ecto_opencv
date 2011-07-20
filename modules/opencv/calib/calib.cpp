#include <ecto/ecto.hpp>
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
    throw std::runtime_error(
                             "The camera_matrix could not be read from the file");
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
}
using namespace calib;
static const char* POINTS = "points_%04d";
static const char* IDEAL = "ideal_%04d";
static const char* FOUND = "found_%04d";

namespace calib
{
struct GatherPoints
{
  typedef std::vector<cv::Point3f> object_pts_t;
  typedef std::vector<cv::Point2f> observation_pts_t;


  static void declare_params(tendrils& params)
  {
    params.declare<int> ("N", "Number of patterns to gather", 2);
  }
  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    int N;
    params.at("N") >> N;
    for(int i =0; i < N; i++)
    {
      in.declare<observation_pts_t> (boost::str(boost::format(POINTS)%i), "Image points");
      in.declare<object_pts_t> (boost::str(boost::format(IDEAL)%i), "The ideal object points.");
      in.declare<bool> (boost::str(boost::format(FOUND)%i));
    }
    out.declare<observation_pts_t> ("out", "The observed pattern points.");
    out.declare<object_pts_t> ("ideal", "The ideal pattern points.");
    out.declare<bool> ("found", "Found some points.");

  }
  void configure(tendrils& params, tendrils& in, tendrils& out)
   {
    params["N"] >> N;
   }
  int process(tendrils& in,tendrils& out)
  {
    object_pts_t obj_pts;
    observation_pts_t observe_pts;
    bool found_any = false;
    for(int i = 0; i < N; i++)
    {
      bool found;
      in[boost::str(boost::format(FOUND)%i)] >> found;
      if(!found) continue;
      found_any = true;
      object_pts_t ideal;
      observation_pts_t points;
      in[boost::str(boost::format(POINTS)%i)] >> points;
      in[boost::str(boost::format(IDEAL)%i)] >> ideal;
      obj_pts.insert(obj_pts.end(),ideal.begin(),ideal.end());
      observe_pts.insert(observe_pts.end(),points.begin(),points.end());
    }
    out["found"] << found_any;
    out["ideal"] << obj_pts;
    out["out"] << observe_pts;
    return ecto::OK;
  }
  int N;
};
}
ECTO_CELL(calib, GatherPoints, "GatherPoints", "Gather points found by multiple patterns.");

namespace calib
{
struct CameraCalibrator
{
  typedef std::vector<cv::Point3f> object_pts_t;
  typedef std::vector<cv::Point2f> observation_pts_t;

  static void declare_params(tendrils& params)
  {
    params.declare<std::string> ("output_file_name",
                                 "The name of the camera calibration file",
                                 "camera.yml");
    params.declare<int> ("n_obs", "Number of observations", 50);
  }

  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    in.declare<observation_pts_t> ("points", "Image points");
    in.declare<object_pts_t> ("ideal", "The ideal object points.");
    in.declare<bool> ("found");
    in.declare<cv::Mat> ("image",
                         "The image that is being used for calibration");
    out.declare<float> ("norm",
                        "Norm of the input points to all previous points observed.");
    out.declare<bool> ("calibrated", "Done calibration", false);
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
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
        std::cout << "distance: " << norm << std::endl << "capturing ..."
            << std::endl;
        object_pts_.push_back(board_pts);
        observation_pts_.push_back(points_in);
      }

    }
    if (int(observation_pts_.size()) > n_obs_ && !calibrated_)
    {
      std::cout << "Calibrating the camera, given " << n_obs_
          << " observations" << std::endl;
      std::vector<cv::Mat> rvecs, tvecs;
      int flags = CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_FIX_PRINCIPAL_POINT;
      camera_.image_size = in.get<cv::Mat> ("image").size();
      double rms = cv::calibrateCamera(object_pts_, observation_pts_,
                                       camera_.image_size, camera_.K,
                                       camera_.D, rvecs, tvecs, flags);
      std::cout << "K = " << camera_.K << std::endl;
      std::cout << "D = " << camera_.D << std::endl;

      std::cout << "camera size = (" << camera_.image_size.width << ", "
          << camera_.image_size.height << ")" << std::endl;

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
}
ECTO_CELL(calib, CameraCalibrator, "CameraCalibrator",                                
          "Accumulates observed points and ideal 3d points, and runs "
          "opencv calibration routines after some number of "
          "satisfactorily unique observations.");

namespace calib
{
struct CameraIntrinsics
{
  static void declare_params(tendrils& params)
  {
    params.declare<std::string> (
                                 "camera_file",
                                 "The camera calibration file. Typically a .yml",
                                 "camera.yml");
  }
  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    out.declare<cv::Size> ("image_size", "The image size.");
    out.declare<cv::Mat> ("K", "3x3 camera intrinsic matrix.");
    out.declare<std::string> ("camera_model",
                              "The camera model. e.g pinhole,...", "pinhole");
  }
  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    readOpenCVCalibration(camera, params.get<std::string> ("camera_file"));
  }
  int process(const tendrils& in, tendrils& out)
  {
    out.get<cv::Mat> ("K") = camera.K;
    out.get<cv::Size> ("image_size") = camera.image_size;
    return 0;
  }
  Camera camera;
};
}

ECTO_CELL(calib, CameraIntrinsics, "CameraIntrinsics",
          "This reads a camera calibration file and puts the results on the outputs.");


namespace calib
{
struct FiducialPoseFinder
{
  typedef std::vector<cv::Point3f> object_pts_t;
  typedef std::vector<cv::Point2f> observation_pts_t;

  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    in.declare<observation_pts_t> ("points", "Image points");
    in.declare<object_pts_t> ("ideal", "The ideal object points.");
    in.declare<cv::Mat> ("K", "The camera projection matrix.",
                         cv::Mat::eye(3, 3, CV_32F));
    in.declare<bool> ("found");
    out.declare<cv::Mat> ("R", "3x3 Rotation matrix.");
    out.declare<cv::Mat> ("T", "3x1 Translation vector.");
  }

  int process(const tendrils& in, tendrils& out)
  {
    if (!in.get<bool> ("found"))
      return 0;
    const observation_pts_t& observation_points =
        in.get<observation_pts_t> ("points");
    const object_pts_t& object_points = in.get<object_pts_t> ("ideal");
    cv::Mat K = in.get<cv::Mat> ("K");
    cv::Mat rvec, tvec;
    cv::solvePnP(object_points, observation_points, K, cv::Mat(), rvec, tvec,
                 false);
    cv::Rodrigues(rvec, out.get<cv::Mat> ("R"));
    out.get<cv::Mat> ("T") = tvec;
    //std::cout << out.get<cv::Mat> ("R") << std::endl << out.get<cv::Mat> ("T") << std::endl;
    return 0;
  }
};
}
ECTO_CELL(calib, FiducialPoseFinder,"FiducialPoseFinder", "Find fiducial pose");
namespace calib
{
struct PoseDrawer
{
  static void draw(cv::Mat& drawImage, const cv::Mat& K, const cv::Mat& R,
                   const cv::Mat T)
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
    Point box_origin(10, drawImage.size().height - 10);
    rectangle(drawImage, box_origin + Point(0,5),
              box_origin + Point(sz.width, -sz.height - 5), Scalar::all(0),
              -1);
    putText(drawImage, scaleText, box_origin, CV_FONT_HERSHEY_SIMPLEX, 1.0,
            c[0], 1, CV_AA, false);
    putText(drawImage, "Z", ip[3], CV_FONT_HERSHEY_SIMPLEX, 1.0, c[3], 1,
            CV_AA, false);
    putText(drawImage, "Y", ip[2], CV_FONT_HERSHEY_SIMPLEX, 1.0, c[2], 1,
            CV_AA, false);
    putText(drawImage, "X", ip[1], CV_FONT_HERSHEY_SIMPLEX, 1.0, c[1], 1,
            CV_AA, false);

  }
  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    in.declare<cv::Mat> ("K", "The camera projection matrix.");
    in.declare<cv::Mat> ("R", "3x3 Rotation matrix.");
    in.declare<cv::Mat> ("T", "3x1 Translation vector.");
    in.declare<cv::Mat> ("image", "The original image to draw the pose onto.");
    in.declare<bool> ("trigger", "Should i draw.",true);
    out.declare<cv::Mat> ("output",
                          "The pose of the fiducial, drawn on an image");
  }

  int process(const tendrils& in, tendrils& out)
  {
    cv::Mat K, R, T, image;
    in.get<cv::Mat>("K").convertTo(K,CV_64F);
    in.get<cv::Mat>("R").convertTo(R,CV_64F);
    in.get<cv::Mat>("T").convertTo(T,CV_64F);

    image = in.get<cv::Mat> ("image");
    cv::Mat& output = out.get<cv::Mat> ("output");
    image.copyTo(output);
    if(in.get<bool>("trigger"))
      draw(output, K, R, T);
    return 0;
  }
};
}
ECTO_CELL(calib, PoseDrawer,"PoseDrawer", "Draw pose");

namespace calib
{
struct PingPongDetector
{
  static void declare_params(tendrils& p)
  {

    p.declare<double> (
                       "dp",
                       "The inverse ratio of the accumulator resolution to the image"
                         " resolution. For example, if dp=1 , the accumulator will have the same resolution as "
                         "the input image, if dp=2 - accumulator will have half as big width and height, etc",
                       2);
    p.declare<double> (
                       "minDist",
                       "Minimum distance between the centers of the detected circles. If the parameter is too small, multiple neighbor "
                         "circles may be falsely detected in addition to a true one. If it is too large, some circles may be missed",
                       10);

    p.declare<double> ("param1",
                       "The first method-specific parameter. in the case of CV_HOUGH_GRADIENT "
                         "it is the higher threshold of the two passed to Canny() edge "
                         "detector (the lower one will be twice smaller)", 200);
    p.declare<double> (
                       "param2",
                       "The second method-specific parameter. in the case of CV_HOUGH_GRADIENT "
                         "it is the accumulator threshold at the center detection stage. The smaller it is, the more false "
                         "circles may be detected. Circles, corresponding to the larger accumulator values, "
                         "will be returned first", 100);
    p.declare<double> ("minRadius", "Min circle radius", 0);
    p.declare<double> ("maxRadius", "The max circle radius.", 0);

  }

  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    in.declare<cv::Mat> ("image",
                         "The grayscale image to search for a calibration pattern in.");
    out.declare<std::vector<cv::Vec3f> > ("circles",
                                          "Detected circles, (x,y,radius).");

  }

  void configure(tendrils& p, tendrils& inputs, tendrils& outputs)
  {
    image_ = inputs.at("image");
    circles_ = outputs.at("circles");
    dp = p.at("dp");
    minDist = p.at("minDist");
    param1 = p.at("param1");
    param2 = p.at("param2");
    minRad = p.at("minRadius");
    maxRad = p.at("maxRadius");
  }

  int process(const tendrils& in, tendrils& out)
  {
    cv::Mat image = image_.read();
    cv::HoughCircles(image, *circles_, CV_HOUGH_GRADIENT, dp.read(), minDist.read(),
                     param1.read(), param2.read(), minRad.read(), maxRad.read());
    return 0;
  }

  ecto::spore<cv::Mat> image_;
  ecto::spore<std::vector<cv::Vec3f> > circles_;
  ecto::spore<double> dp, minDist, param1, param2, minRad, maxRad;

};
}
ECTO_CELL(calib, PingPongDetector, "PingPongDetector",
          "Detect 40 mm ping pong balls.");
