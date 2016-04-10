#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/filesystem.hpp>

#include <vector>

using namespace ecto;
using std::vector;
namespace calib
{
struct SubrectRectifier
{
  static void declare_params(tendrils& p)
  {
    p.declare<float>("xsize_world", "x size of extracted rectangle in world meters", 0.1);
    p.declare<float>("ysize_world", "y size of extracted rectangle in world meters", 0.1);

    p.declare<unsigned>("xsize_pixels", "x size of extracted image in pixels", 250);
    p.declare<unsigned>("ysize_pixels", "y size of extracted image in pixels", 250);

    p.declare<float>("xoffset", "x offset from input pose in world meters", 0.0);
    p.declare<float>("yoffset", "y offset from input pose in world meters", 0.0);
    p.declare<float>("zoffset", "z offset from input pose in world meters", 0.0);
    
  }

  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    in.declare<cv::Mat> ("K", "The camera projection matrix.");
    in.declare<cv::Mat> ("R", "3x3 Rotation matrix.");
    in.declare<cv::Mat> ("T", "3x1 Translation vector.");
    in.declare<cv::Mat> ("image", "input image");
    out.declare<cv::Mat> ("output", "Output extracted rectified rectangle");
  }

  void configure(const tendrils& p, const tendrils& inputs, const tendrils& o)
  {
    xoffset = p["xoffset"];
    yoffset = p["yoffset"];
    zoffset = p["zoffset"];
    
    xsize_world = p["xsize_world"];
    ysize_world = p["ysize_world"];

    xsize_pixels = p["xsize_pixels"];
    ysize_pixels = p["ysize_pixels"];

    output = o["output"];

  }

  int process(const tendrils& in, const tendrils& out)
  {
    using namespace cv;

    cv::Mat K, R, T, image;
    in.get<cv::Mat>("K").convertTo(K,CV_64F);
    in.get<cv::Mat>("R").convertTo(R,CV_64F);
    in.get<cv::Mat>("T").convertTo(T,CV_64F);

    //std::cout << "K:" << K << "\n";
    //std::cout << "R:" << R << "\n";
    //std::cout << "T:" << T << "\n";

    if (R.rows == 0 || R.cols == 0 || T.rows == 0 || T.cols == 0)
      {
        *output = cv::Mat(*xsize_pixels, *ysize_pixels, image.type());
        return 0;
      }

    image = in.get<cv::Mat> ("image");

    image.copyTo(*output);

    vector<Point3f> worldcorners;
    worldcorners.push_back(Point3f(*xoffset, *yoffset + *ysize_world, *zoffset));
    worldcorners.push_back(Point3f(*xoffset + *xsize_world, *yoffset + *ysize_world, *zoffset));
    worldcorners.push_back(Point3f(*xoffset + *xsize_world, *yoffset, *zoffset));
    worldcorners.push_back(Point3f(*xoffset, *yoffset, *zoffset));

    vector<Point2f> imagecorners;
    projectPoints(Mat(worldcorners), R, T, K, Mat(4, 1, CV_64FC1, Scalar(0)), imagecorners);
    Scalar white(255,255,255);

    //line(*output, imagecorners[0], imagecorners[1], white);
    //line(*output, imagecorners[1], imagecorners[2], white);
    //line(*output, imagecorners[2], imagecorners[3], white);
    //line(*output, imagecorners[3], imagecorners[0], white);

    vector<Point2f> dstcorners;
    dstcorners.push_back(Point2f(0, 0));
    dstcorners.push_back(Point2f(*xsize_pixels, 0));
    dstcorners.push_back(Point2f(*xsize_pixels, *ysize_pixels));
    dstcorners.push_back(Point2f(0, *ysize_pixels));

    cv::Mat H = findHomography(cv::Mat(imagecorners), cv::Mat(dstcorners));
    warpPerspective(image, *output, H, Size(*xsize_pixels, *ysize_pixels));
    return 0;
  }

  ecto::spore<float> xoffset, yoffset, zoffset, xsize_world, ysize_world;
  ecto::spore<unsigned> xsize_pixels, ysize_pixels;
  ecto::spore<cv::Mat> output;
};
}
ECTO_CELL(calib, calib::SubrectRectifier, "SubrectRectifier", "Pull a trapezoid out of an image and rectify");

