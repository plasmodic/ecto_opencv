#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/filesystem.hpp>

using namespace ecto;

struct SubrectRectifier
{
  static void declare_params(tendrils& p)
  {
    p.declare<float>("xsize_world", "x size of extracted rectangle in world meters", 0.1);
    p.declare<float>("ysize_world", "y size of extracted rectangle in world meters", 0.1);

    p.declare<int>("xsize_pixels", "x size of extracted image in pixels", 250);
    p.declare<int>("ysize_pixels", "y size of extracted image in pixels", 250);

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

  void configure(tendrils& p, tendrils& inputs, tendrils& o)
  {
    xoffset = p.at("xoffset");
    yoffset = p.at("yoffset");
    zoffset = p.at("zoffset");
    
    xsize_world = p.at("xsize_world");
    ysize_world = p.at("ysize_world");

    output = o.at("output");

  }

  static void draw(cv::Mat& drawImage, const cv::Mat& K, const cv::Mat& R,
                   const cv::Mat T)
  {
    /*
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
    */
  }
  int process(const tendrils& in, tendrils& out)
  {
    using namespace cv;

    cv::Mat K, R, T, image;
    in.get<cv::Mat>("K").convertTo(K,CV_64F);
    in.get<cv::Mat>("R").convertTo(R,CV_64F);
    in.get<cv::Mat>("T").convertTo(T,CV_64F);

    std::cout << "K:" << K << "\n";
    std::cout << "R:" << R << "\n";
    std::cout << "T:" << T << "\n";

    image = in.get<cv::Mat> ("image");
    image.copyTo(*output);

    vector<Point3f> worldcorners;
    worldcorners.push_back(Point3f(*xoffset, *yoffset, *zoffset));
    worldcorners.push_back(Point3f(*xoffset + *xsize_world, *yoffset, *zoffset));
    worldcorners.push_back(Point3f(*xoffset + *xsize_world, *yoffset + *ysize_world, *zoffset));
    worldcorners.push_back(Point3f(*xoffset, *yoffset + *ysize_world, *zoffset));

    vector<Point2f> imagecorners;
    projectPoints(Mat(worldcorners), R, T, K, Mat(4, 1, CV_64FC1, Scalar(0)), imagecorners);
    Scalar white(255,255,255);

    line(*output, imagecorners[0], imagecorners[1], white);
    line(*output, imagecorners[1], imagecorners[2], white);
    line(*output, imagecorners[2], imagecorners[3], white);
    line(*output, imagecorners[3], imagecorners[0], white);

    return 0;
  }

  ecto::spore<float> xoffset, yoffset, zoffset, xsize_world, ysize_world;
  ecto::spore<cv::Mat> output;
};
ECTO_CELL(calib, SubrectRectifier, "SubrectRectifier", "Pull a trapezoid out of an image and recitfy");

