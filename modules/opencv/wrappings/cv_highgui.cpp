#include <boost/python.hpp>
#include <boost/python/overloads.hpp>
#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>

namespace bp = boost::python;

namespace
{
  BOOST_PYTHON_FUNCTION_OVERLOADS(imread_overloads,cv::imread,1,2)
  ;

  BOOST_PYTHON_FUNCTION_OVERLOADS(imwrite_overloads,cv::imwrite,2,3)
  ;
  BOOST_PYTHON_FUNCTION_OVERLOADS(imencode_overloads,cv::imencode,3,4)
  ;
}
namespace opencv_wrappers
{
  void wrap_highgui_defines();
  void wrap_video_capture()
  {
    bp::class_<cv::VideoCapture> VideoCapture_("VideoCapture");
    VideoCapture_.def(bp::init<>());
    VideoCapture_.def(bp::init<std::string>());
    VideoCapture_.def(bp::init<int>());
    typedef bool(cv::VideoCapture::*open_1)(const std::string&);
    typedef bool(cv::VideoCapture::*open_2)(int);
    VideoCapture_.def("open", open_1(&cv::VideoCapture::open));
    VideoCapture_.def("open", open_2(&cv::VideoCapture::open));
    VideoCapture_.def("isOpened", &cv::VideoCapture::isOpened);
    VideoCapture_.def("release", &cv::VideoCapture::release);
    VideoCapture_.def("grab", &cv::VideoCapture::grab);
    VideoCapture_.def("retrieve", &cv::VideoCapture::retrieve);
    VideoCapture_.def("read", &cv::VideoCapture::read);
    VideoCapture_.def("set", &cv::VideoCapture::set);
    VideoCapture_.def("get", &cv::VideoCapture::get);
  }

  void wrap_video_writer()
  {
    bp::class_<cv::VideoWriter> VideoWriter_("VideoWriter");
    VideoWriter_.def(bp::init<>());
    VideoWriter_.def(bp::init<const std::string&, int, double, cv::Size, bool>());
    VideoWriter_.def("open", &cv::VideoWriter::open);
    VideoWriter_.def("isOpened", &cv::VideoWriter::isOpened);
    VideoWriter_.def("write", &cv::VideoWriter::write);
  }

  void depth2points(const cv::Mat& depth, cv::Mat& points)
  {
    const float f = 575;
    const float finv = 1 / f;
    const float cx = depth.size().width / 2;
    const float cy = depth.size().height / 2;
    points.create(depth.size(), CV_32FC3);
    cv::Mat_<cv::Point3f>::iterator itp = points.begin<cv::Point3f> ();
    cv::Mat_<uint16_t>::const_iterator it = depth.begin<uint16_t> ();
    for (int y = 0; y < 480; y++)
    {
      for (int x = 0; x < 640; x++, ++it, ++itp)
      {
        const float Z = *it * 0.001f;
        cv::Point3f& p = *itp;
        p.z = Z;
        p.x = (x - cx) * Z * finv;
        p.y = (y - cy) * Z * finv;
      }
    }
  }

  void savePoints(std::string filename, const cv::Mat& points)
  {
    cv::Mat_<cv::Point3f>::const_iterator itp = points.begin<cv::Point3f> (), end = points.end<cv::Point3f> ();
    std::ofstream out(filename.c_str());
    out << "ply\n"
      "format ascii 1.0\n"
      "comment made by anonymous  { comments keyword specified, like all lines }\n"
      "comment this file is a cube\n"
      "element vertex " << points.size().area()<<"          { define vertex element, 8 of them in file }\n"
      "property float32 x         { vertex contains float x coordinate }\n"
      "property float32 y         { y coordinate is also a vertex property }\n"
      "property float32 z         { z coordinate, too }\n"
      "end_header                 { delimits the end of the header }\n";

    while (itp != end)
    {
      const cv::Point3f& p = *itp;
      out << p.x << " " << p.y << " " << p.z << "\n";
      ++itp;
    }

  }
  void wrap_highgui()
  {
    wrap_highgui_defines();
    //video stuff.
    wrap_video_capture();
    wrap_video_writer();
    //image windows
    bp::def("imshow", cv::imshow);
    bp::def("waitKey", cv::waitKey);
    bp::def("namedWindow", cv::namedWindow);
    //image io
    bp::def("imread", cv::imread, imread_overloads());
    bp::def("imwrite", cv::imwrite, imwrite_overloads());
    bp::def("imdecode", cv::imdecode);

    bp::def("imencode", cv::imencode, imencode_overloads());
    bp::def("depth2points", depth2points);
    bp::def("savepoints",savePoints);
  }
}
