#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/stl_iterator.hpp>

#include <string>

#include <opencv2/core/core.hpp>

namespace bp = boost::python;

namespace
{

  template<typename T>
  struct PointWrapper
  {
    typedef cv::Point_<T> Point_t;
    typedef std::vector<Point_t> VectorPoint_t;

    static Point_t make_point(bp::object o)
    {
      T x = bp::extract<T>(o[0]);
      T y = bp::extract<T>(o[1]);
      return  Point_t(x,y);
    }

    static boost::shared_ptr<Point_t> make_point_ptr(bp::object o)
    {
      return boost::shared_ptr<Point_t>(new Point_t(make_point(o)));
    }

    static boost::shared_ptr<VectorPoint_t> make_points(bp::object o)
    {
      size_t l = bp::len(o);
      boost::shared_ptr<VectorPoint_t> v(new VectorPoint_t(l));
      for(size_t i = 0; i < l; i++){
        (*v)[i] = make_point(o[i]);
      }
      return v;
    }

    static void wrap_point(const std::string& name)
    {
      typedef cv::Point_<T> Point_t;
      bp::class_<Point_t,boost::shared_ptr<Point_t> > Point_(name.c_str());
      Point_.def(bp::init<>());
      Point_.def(bp::init<T, T>());
      Point_.def(bp::init<Point_t>());
      Point_.def("__init__",bp::make_constructor(&make_point_ptr));
      Point_.def_readwrite("x", &Point_t::x);
      Point_.def_readwrite("y", &Point_t::y);
      Point_.def_readwrite("dot", &Point_t::dot);
      Point_.def_readwrite("inside", &Point_t::inside);

      //bp::implicitly_convertible<bp::object,Point_t>();
      std::string vec_name = "Vector"+name;
      bp::class_<VectorPoint_t, boost::shared_ptr<VectorPoint_t> > (vec_name.c_str())
        .def(bp::vector_indexing_suite<VectorPoint_t>())
        .def("__init__", bp::make_constructor(&make_points));

      //      bp::implicitly_convertible<bp::object,VectorPoint_t>();
    }
  };

template<typename T>
  void wrap_rect(const std::string& name)
  {
    typedef cv::Rect_<T> Rect_t;
    bp::class_<Rect_t> c_(name.c_str());
    c_.def(bp::init<>());
    c_.def(bp::init<T, T, T, T>());
    c_.def(bp::init<cv::Point_<T>, cv::Point_<T> >());
    c_.def(bp::init<cv::Point_<T>, cv::Size_<T> >());

    c_.def(bp::init<Rect_t>());
    c_.def_readwrite("x", &Rect_t::x);
    c_.def_readwrite("y", &Rect_t::y);
    c_.def_readwrite("width", &Rect_t::width);
    c_.def_readwrite("height", &Rect_t::height);
    c_.def("tl", &Rect_t::tl);
    c_.def("br", &Rect_t::br);
    c_.def("size", &Rect_t::size);
    c_.def("area", &Rect_t::area);
    c_.def("contains", &Rect_t::contains);
  }
}

namespace opencv_wrappers
{
  void wrap_points()
  {
    bp::class_<cv::Size> Size_("Size");
    Size_.def(bp::init<int, int>());
    Size_.def_readwrite("width", &cv::Size::width);
    Size_.def_readwrite("height", &cv::Size::height);
    Size_.def("area", &cv::Size::area);

    PointWrapper<int>::wrap_point("Point");
    PointWrapper<float>::wrap_point("Point2f");
    PointWrapper<double>::wrap_point("Point2d");

    wrap_rect<int> ("Rect");
    wrap_rect<float> ("Rectf");
    wrap_rect<double> ("Rectd");

  }
}
