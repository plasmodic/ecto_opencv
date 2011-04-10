#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

#include <string>

#include <opencv2/core/core.hpp>

namespace bp = boost::python;

namespace
{

  template<typename T>
    void mat_set_t(cv::Mat&m, bp::object o)
    {

      int length = bp::len(o);
      if (m.size().area() != length || m.depth() != cv::DataType<T>::depth)
      {
        m.create(length, 1, cv::DataType<T>::type);
      }
      bp::stl_input_iterator<T> begin(o), end;
      typename cv::Mat_<T>::iterator it = m.begin<T> (), itEnd = m.end<T> ();
      for (; it != itEnd; ++it)
        *it = *(begin++);
    }

  void mat_set(cv::Mat& m, bp::object o, int type)
  {
    //switch on the given type and use this type as the cv::Mat element type
    switch (CV_MAT_DEPTH(type))
    {
      case CV_8U:
        mat_set_t<unsigned char> (m, o);
        break;
      case CV_8S:
        mat_set_t<signed char> (m, o);
        break;
      case CV_16U:
        mat_set_t<uint16_t> (m, o);
        break;
      case CV_16S:
        mat_set_t<int16_t> (m, o);
        break;
      case CV_32S:
        mat_set_t<int32_t> (m, o);
        break;
      case CV_32F:
        mat_set_t<float_t> (m, o);
        break;
      case CV_64F:
        mat_set_t<double_t> (m, o);
        break;
      default:
        throw std::logic_error("Given type not supported.");
    }
  }
  cv::Size mat_size(cv::Mat& m)
  {
    return m.size();
  }

  int mat_type(cv::Mat& m)
  {
    return m.type();
  }
  void mat_set(cv::Mat& m, bp::object o)
  {
    if (m.empty())
      throw std::logic_error("The matrix is empty, can not deduce type.");
    //use the m.type and implicitly assume that o is of this type
    mat_set(m, o, m.type());
  }

  //overloaded function pointers
  void (*mat_set_p2)(cv::Mat&, bp::object) = mat_set;
  void (*mat_set_p3)(cv::Mat&, bp::object, int) = mat_set;

}
namespace opencv_wrappers
{
  void wrap_mat()
  {
    //mat definition
    bp::class_<cv::Mat> Mat_("Mat");
    Mat_.def(bp::init<>());
    Mat_.def(bp::init<int, int, int>());
    Mat_.def(bp::init<cv::Size, int>());
    Mat_.def_readonly("rows", &cv::Mat::rows, "the number of rows");
    Mat_.def_readonly("cols", &cv::Mat::cols, "the number of columns");
    Mat_.def("row", &cv::Mat::row, "get the row at index");
    Mat_.def("col", &cv::Mat::col, "get the column at index");
    Mat_.def("fromarray", mat_set_p2, "Set a Matrix from a python iterable. Assumes the type of the Mat "
      "while setting. If the size of the Matrix will not accommodate "
      "the given python iterable length, then the matrix will be allocated "
      "as a single channel, Nx1 vector where N = len(list)");
    Mat_.def("fromarray", mat_set_p3, "Set a Matrix from a python array. Explicitly give "
      "the type of the array. If the size of the Matrix will not accommodate "
      "the given python iterable length, then the matrix will be allocated "
      "as a single channel, Nx1 vector where N = len(list)");
    Mat_.def("size", mat_size);
    Mat_.def("type", mat_type);
    Mat_.def("clone", &cv::Mat::clone);

  }
}
