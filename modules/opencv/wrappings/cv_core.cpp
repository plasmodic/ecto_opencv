#include <boost/foreach.hpp>

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>

#include <opencv2/core/core.hpp>

namespace bp = boost::python;

namespace{

template<typename T>
  void mat_set_t(cv::Mat&m, bp::object o)
  {

    int length = bp::len(o);
    CV_Assert(length == m.size().area())
      ;
    bp::stl_input_iterator<T> begin(o), end;
    typename cv::Mat_<T>::iterator it = m.begin<T> (), itEnd = m.end<T> ();
    for (; it != itEnd; ++it)
      *it = *(begin++);
  }

void mat_set(cv::Mat& m, bp::object o, int type)
{
  //switch on the given type and use this type as the cv::Mat element type
  switch (type)
  {
    case cv::DataType<unsigned char>::type:
      mat_set_t<unsigned char> (m, o);
      break;
    case cv::DataType<int>::type:
      mat_set_t<int> (m, o);
      break;
    case cv::DataType<float>::type:
      mat_set_t<float> (m, o);
      break;
    case cv::DataType<double>::type:
      mat_set_t<double> (m, o);

      break;
  }

}
cv::Size mat_size(const cv::Mat&m){
	return m.size();
}
void mat_set(cv::Mat& m, bp::object o)
{
	//use the m.type and implicitly assume that o is of this type
	mat_set (m,o,m.type());
}

//overloaded function pointers
void  (*mat_set_p2)(cv::Mat&, bp::object) = mat_set;
void  (*mat_set_p3)(cv::Mat&, bp::object, int) = mat_set;

}


namespace opencv_wrappers
{
  void wrap_cv_core()
  {
    //mat definition
   bp::class_<cv::Mat>("Mat")
       .def(bp::init<>())
       .def(bp::init<int, int, int>())
       .def(bp::init<cv::Size, int>())
       .def_readonly("rows", &cv::Mat::rows, "the number of rows")
       .def_readonly("cols",&cv::Mat::cols, "the number of columns")
       .def("row",&cv::Mat::row, "get the row at index")
       .def("col",&cv::Mat::col, "get the column at index")
       .def("fromarray",mat_set_p2)
       .def("fromarray",mat_set_p3)
       .def("size",&mat_size)
       ;
  //define opencv consts
  bp::object opencv = bp::scope();
  opencv.attr("CV_8UC1")  = CV_8UC1;
  opencv.attr("CV_8UC3")  = CV_8UC3;
  opencv.attr("CV_32SC1")  = CV_32SC1;
  opencv.attr("CV_32FC1")  = CV_32FC1;
  opencv.attr("CV_64FC1")  = CV_64FC1;
  
  bp::class_<cv::Size>("Size")
       .def(bp::init<int, int>())
       .def_readwrite("width", &cv::Size::width)
       .def_readwrite("height",&cv::Size::height)
       .def("area",&cv::Size::area)
       ;
  }
}
