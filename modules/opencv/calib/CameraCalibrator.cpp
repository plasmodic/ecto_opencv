#include <ecto/ecto.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "calib.hpp"

using ecto::tendrils;

namespace calib
{
  struct CameraCalibrator
  {
    typedef std::vector<cv::Point3f> object_pts_t;
    typedef std::vector<cv::Point2f> observation_pts_t;

    static void
    declare_params(tendrils& params)
    {
      params.declare<std::string>("output_file_name", "The name of the camera calibration file", "camera.yml");
      params.declare<int>("n_obs", "Number of observations", 50);
      params.declare<bool>("quit_when_calibrated", "return QUIT from process once calibration done", true);
    }

    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare<observation_pts_t>("points", "Image points");
      in.declare<object_pts_t>("ideal", "The ideal object points.");
      in.declare<bool>("found");
      in.declare<cv::Mat>("image", "The image that is being used for calibration");
      out.declare<float>("norm", "Norm of the input points to all previous points observed.");
      out.declare<bool>("calibrated", "Done calibration", false);
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      n_obs_ = params.get<int>("n_obs");
      camera_output_file_ = params.get<std::string>("output_file_name");
      object_pts_.clear();
      norm_thresh_ = 150; //pixel values;
      calibrated_ = false;
      quit_when_calibrated_ = params.get<bool>("quit_when_calibrated");
    }

    double
    calcDistance(const observation_pts_t& in) const
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
    int
    process(const tendrils& in, const tendrils& out)
    {
      const observation_pts_t& points_in = in.get<observation_pts_t>("points");
      const object_pts_t& board_pts = in.get<object_pts_t>("ideal");
      bool found = in.get<bool>("found");
      float norm = 0;
      if (calibrated_)
        return ecto::OK;

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
        int flags = CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_FIX_PRINCIPAL_POINT | CV_CALIB_ZERO_TANGENT_DIST;
        camera_.image_size = in.get<cv::Mat>("image").size();
        double rms = cv::calibrateCamera(object_pts_, observation_pts_, camera_.image_size, camera_.K, camera_.D, rvecs,
                                         tvecs, flags);
        std::cout << "K = " << camera_.K << std::endl;
        std::cout << "D = " << camera_.D << std::endl;

        std::cout << "camera size = (" << camera_.image_size.width << ", " << camera_.image_size.height << ")"
        << std::endl;

        writeOpenCVCalibration(camera_, camera_output_file_);

        printf("RMS error reported by calibrateCamera: %g\n", rms);
        calibrated_ = true;
        if (quit_when_calibrated_)
          return ecto::QUIT;
      }

      out.get<float>("norm") = norm;
      out.get<bool>("calibrated") = calibrated_;
      return ecto::OK;
    }
    cv::Size grid_size_;
    int n_obs_;
    float norm_thresh_;
    bool calibrated_;
    bool quit_when_calibrated_;
    std::vector<object_pts_t> object_pts_;
    std::vector<observation_pts_t> observation_pts_;
    Camera camera_;
    std::string camera_output_file_;
  };
}
ECTO_CELL(calib, calib::CameraCalibrator, "CameraCalibrator", "Accumulates observed points and ideal 3d points, and runs "
"opencv calibration routines after some number of "
"satisfactorily unique observations.");
