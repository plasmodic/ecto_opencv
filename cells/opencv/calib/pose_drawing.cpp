#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "calib.hpp"
#include <boost/format.hpp>

using ecto::tendrils;
using std::string;
using std::vector;

namespace calib
{
  struct PoseDrawer
  {
    static void
    draw(cv::Mat& drawImage, const cv::Mat& K, const cv::Mat& R, const cv::Mat T)
    {
      using namespace cv;

      if (R.empty() || T.empty())
        return;
      float scale = 0.1;
      Point3f z(0, 0, scale);
      Point3f x(scale, 0, 0);
      Point3f y(0, scale, 0);
      Point3f o(0, 0, 0);
      vector<Point3f> op(4);
      op[1] = x, op[2] = y, op[3] = z, op[0] = o;
      vector<Point2f> ip;
      projectPoints(Mat(op), R, T, K, Mat(4, 1, CV_64FC1, Scalar(0)), ip);

      vector<Scalar> c(4); //colors
      c[0] = Scalar(255, 255, 255);
      c[1] = Scalar(255, 0, 0); //x
      c[2] = Scalar(0, 255, 0); //y
      c[3] = Scalar(0, 0, 255); //z
      line(drawImage, ip[0], ip[1], c[1]);
      line(drawImage, ip[0], ip[2], c[2]);
      line(drawImage, ip[0], ip[3], c[3]);
      string scaleText = boost::str(boost::format("scale %0.2f meters")%scale);
      int baseline = 0;
      Size sz = cv::getTextSize(scaleText, CV_FONT_HERSHEY_SIMPLEX, 1, 1, &baseline);
      Point box_origin(10, drawImage.size().height - 10);
      rectangle(drawImage, box_origin + Point(0, 5), box_origin + Point(sz.width, -sz.height - 5), Scalar::all(0), -1);
      putText(drawImage, scaleText, box_origin, CV_FONT_HERSHEY_SIMPLEX, 1.0, c[0], 1, CV_AA, false);
      putText(drawImage, "Z", ip[3], CV_FONT_HERSHEY_SIMPLEX, 1.0, c[3], 1, CV_AA, false);
      putText(drawImage, "Y", ip[2], CV_FONT_HERSHEY_SIMPLEX, 1.0, c[2], 1, CV_AA, false);
      putText(drawImage, "X", ip[1], CV_FONT_HERSHEY_SIMPLEX, 1.0, c[1], 1, CV_AA, false);

    }
    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare<cv::Mat>("K", "The camera projection matrix.");
      in.declare<cv::Mat>("R", "3x3 Rotation matrix.");
      in.declare<cv::Mat>("T", "3x1 Translation vector.");
      in.declare<cv::Mat>("image", "The original image to draw the pose onto.");
      in.declare<bool>("trigger", "Should i draw.", true);
      out.declare<cv::Mat>("output", "The pose of the fiducial, drawn on an image");
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      cv::Mat K, R, T, image;
      in.get<cv::Mat>("K").convertTo(K, CV_64F);
      in.get<cv::Mat>("R").convertTo(R, CV_64F);
      in.get<cv::Mat>("T").convertTo(T, CV_64F);

      image = in.get<cv::Mat>("image");
      cv::Mat& output = out.get<cv::Mat>("output");
      output = cv::Mat();
      image.copyTo(output);
      if (in.get<bool>("trigger"))
        draw(output, K, R, T);
      return 0;
    }
  };

  struct PosesDrawer
  {
    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare<cv::Mat>("K", "The camera projection matrix.");
      in.declare<std::vector<cv::Mat> >("Rs", "3x3 Rotation matrix.");
      in.declare<std::vector<cv::Mat> >("Ts", "3x1 Translation vector.");
      in.declare<cv::Mat>("image", "The original image to draw the pose onto.");
      in.declare<bool>("trigger", "Should i draw.", true);
      out.declare<cv::Mat>("output", "The pose of the fiducial, drawn on an image");
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      const cv::Mat & image = in.get<cv::Mat>("image");
      cv::Mat & output = out.get<cv::Mat>("output");

      image.copyTo(output);
      if (in.get<bool>("trigger"))
      {
        std::vector<cv::Mat> Rs = in.get<std::vector<cv::Mat> >("Rs"), Ts = in.get<std::vector<cv::Mat> >("Ts");
        cv::Mat K, R, T;
        in.get<cv::Mat>("K").convertTo(K, CV_64F);

        for (unsigned int i = 0; i < Rs.size(); ++i)
        {
          Rs[i].convertTo(R, CV_64F);
          Ts[i].convertTo(T, CV_64F);
          PoseDrawer::draw(output, K, R, T);
        }
      }
      return 0;
    }
  };
}
using namespace calib;
ECTO_CELL(calib, PoseDrawer, "PoseDrawer", "Draw pose");
ECTO_CELL(calib, PosesDrawer, "PosesDrawer", "Draw poses");
