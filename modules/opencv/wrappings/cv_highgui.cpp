#include <boost/python.hpp>

#include <opencv2/highgui/highgui.hpp>

namespace bp = boost::python;

namespace
{

}
namespace opencv_wrappers
{
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

  void wrap_highgui()
  {
    bp::object opencv = bp::scope();
    opencv.attr("CV_WINDOW_AUTOSIZE") = int(CV_WINDOW_AUTOSIZE);
    opencv.attr("CV_CAP_ANY") = int(CV_CAP_ANY);
    opencv.attr("CV_CAP_MIL") = int(CV_CAP_MIL);
    opencv.attr("CV_CAP_VFW") = int(CV_CAP_VFW);
    opencv.attr("CV_CAP_V4L") = int(CV_CAP_V4L);
    opencv.attr("CV_CAP_V4L2") = int(CV_CAP_V4L2);
    opencv.attr("CV_CAP_FIREWARE") = int(CV_CAP_FIREWARE);
    opencv.attr("CV_CAP_FIREWIRE") = int(CV_CAP_FIREWIRE);
    opencv.attr("CV_CAP_IEEE1394") = int(CV_CAP_IEEE1394);
    opencv.attr("CV_CAP_DC1394") = int(CV_CAP_DC1394);
    opencv.attr("CV_CAP_CMU1394") = int(CV_CAP_CMU1394);
    opencv.attr("CV_CAP_STEREO") = int(CV_CAP_STEREO);
    opencv.attr("CV_CAP_TYZX") = int(CV_CAP_TYZX);
    opencv.attr("CV_TYZX_LEFT") = int(CV_TYZX_LEFT);
    opencv.attr("CV_TYZX_RIGHT") = int(CV_TYZX_RIGHT);
    opencv.attr("CV_TYZX_COLOR") = int(CV_TYZX_COLOR);
    opencv.attr("CV_TYZX_Z") = int(CV_TYZX_Z);
    opencv.attr("CV_CAP_QT") = int(CV_CAP_QT);
    opencv.attr("CV_CAP_UNICAP") = int(CV_CAP_UNICAP);
    opencv.attr("CV_CAP_DSHOW") = int(CV_CAP_DSHOW);
    opencv.attr("CV_CAP_PVAPI") = int(CV_CAP_PVAPI);
    opencv.attr("CV_CAP_OPENNI") = int(CV_CAP_OPENNI);

    //video stuff.
    wrap_video_capture();
    wrap_video_writer();
    //image windows
    bp::def("imshow", cv::imshow);
    bp::def("waitKey", cv::waitKey);
    bp::def("namedWindow", cv::namedWindow);
    //image io
    bp::def("imread", cv::imread);
    bp::def("imwrite", cv::imwrite);
    bp::def("imdecode", cv::imdecode);
    bp::def("imencode", cv::imencode);
  }
}
