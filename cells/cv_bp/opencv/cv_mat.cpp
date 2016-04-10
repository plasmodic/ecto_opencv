#define PY_ARRAY_UNIQUE_SYMBOL PyArrayHandle
#include <boost/python.hpp>
#include <numpy/arrayobject.h>
#include <boost/python/stl_iterator.hpp>
#include <boost/format.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <string>

#include <opencv2/core/core.hpp>

namespace bp = boost::python;

namespace
{

  template<typename T>
  inline void
  mat_set_t(cv::Mat&m, bp::object o)
  {

    int length = bp::len(o);
    if (m.size().area() != length || m.depth() != cv::DataType<T>::depth)
    {
      m.create(length, 1, cv::DataType<T>::type);
    }
    bp::stl_input_iterator<T> begin(o), end;
    typename cv::Mat_<T>::iterator it = m.begin<T>(), itEnd = m.end<T>();
    for (; it != itEnd; ++it)
      *it = *(begin++);
  }

  namespace numpy_helpers
  {
    PyArray_TYPES
    fromCVDepth(int cv_depth)
    {
      switch (cv_depth)
      {
        case CV_8U:
          return PyArray_UBYTE;
        case CV_8S:
          return PyArray_BYTE;
        case CV_16S:
          return PyArray_INT16;
        case CV_16U:
          return PyArray_UINT16;
        case CV_32S:
          return PyArray_INT32;
        case CV_32F:
          return PyArray_FLOAT32;
        case CV_64F:
          return PyArray_FLOAT64;
        default:
          throw std::runtime_error("Unsupported cv::depth.");
      }
    }

    int
    fromPyArray_TYPES(PyArray_TYPES py_type)
    {
      switch (py_type)
      {
        case PyArray_UBYTE:
          return CV_8U;
        case PyArray_BYTE:
          return CV_8S;
        case PyArray_INT16:
          return CV_16S;
        case PyArray_UINT16:
          return CV_16U;
        case PyArray_INT32:
          return CV_32S;
        case PyArray_FLOAT32:
          return CV_32F;
        case PyArray_FLOAT64:
          return CV_64F;
        default:
          throw std::runtime_error(boost::str(boost::format("Unsupported py type. Type: %d")%py_type));
      }
    }

    //Create a Numeric array with dimensions dimens and Numeric type t
    bp::numeric::array
    makeNum(std::vector<int> dimens, PyArray_TYPES t = PyArray_DOUBLE)
    {
      using namespace bp;
      object obj(handle<>(PyArray_FromDims(dimens.size(), &dimens[0], t)));
      return extract<numeric::array>(obj);
    }

    void*
    data(bp::numeric::array arr)
    {
      if (!PyArray_Check(arr.ptr()))
      {
        PyErr_SetString(PyExc_ValueError, "expected a PyArrayObject");
        bp::throw_error_already_set();
      }
      return PyArray_DATA(arr.ptr());
    }

    //Copy data into the array
    void
    copy_data(boost::python::numeric::array arr, const uchar* new_data)
    {
      uchar* arr_data = (uchar*) data(arr);
      size_t nbytes = PyArray_NBYTES(arr.ptr());
      for (size_t i = 0; i < nbytes; i++)
      {
        arr_data[i] = new_data[i];
      }
    }

    //Copy data into the array
    void
    copy_data(uchar* new_data, boost::python::numeric::array arr)
    {
      uchar* arr_data = (uchar*) data(arr);
      size_t nbytes = PyArray_NBYTES(arr.ptr());
      for (size_t i = 0; i < nbytes; i++)
      {
        new_data[i] = arr_data[i];
      }
    }
    PyArray_TYPES
    type(bp::numeric::array arr)
    {
      return PyArray_TYPES(PyArray_TYPE(arr.ptr()));
    }

    //Return the number of dimensions
    int
    rank(bp::numeric::array arr)
    {
      using namespace bp;
      //std::cout << "inside rank" << std::endl;
      if (!PyArray_Check(arr.ptr()))
      {
        PyErr_SetString(PyExc_ValueError, "expected a PyArrayObject");
        throw_error_already_set();
      }
      return PyArray_NDIM(arr.ptr());
    }

    std::vector<npy_intp>
    shape(bp::numeric::array arr)
    {
      using namespace bp;
      std::vector<npy_intp> out_dims;
      if (!PyArray_Check(arr.ptr()))
      {
        PyErr_SetString(PyExc_ValueError, "expected a PyArrayObject");
        throw_error_already_set();
      }
      npy_intp* dims_ptr = PyArray_DIMS(arr.ptr());
      int the_rank = rank(arr);
      for (int i = 0; i < the_rank; i++)
      {
        out_dims.push_back(*(dims_ptr + i));
      }
      return out_dims;
    }
  }

  void
  mat_from_array(cv::Mat& m, bp::numeric::array array)
  {
    std::vector<npy_intp> shape = numpy_helpers::shape(array);
    int cv_depth = numpy_helpers::fromPyArray_TYPES(numpy_helpers::type(array));
    if(shape.size() != 2 && shape.size() != 3)
      throw std::runtime_error("Can't handle numpy arrays unless they are of dimensionality of 2 or 3");
    int channels = shape.size() == 2? 1: shape[2];
    int cv_type = CV_MAKETYPE(cv_depth, channels);
    m.create(shape[0], shape[1], cv_type);
    numpy_helpers::copy_data(m.ptr(0), array);
  }

  bp::object
  mat_to_array(cv::Mat& m)
  {
    typedef bp::numeric::array array_t;
    std::vector<int> dim(2);
    dim[0] = m.rows;
    dim[1] = m.cols;
    if(m.channels() != 1)
      dim.push_back(m.channels());
    bp::numeric::array array(numpy_helpers::makeNum(dim, numpy_helpers::fromCVDepth(m.depth())));
    numpy_helpers::copy_data(array, m.ptr(0));
    return array;
  }

  inline void
  mat_set(cv::Mat& m, bp::object o, int type)
  {
    //switch on the given type and use this type as the cv::Mat element type
    switch(CV_MAT_DEPTH(type)){
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
  inline cv::Size
  mat_size(cv::Mat& m)
  {
    return m.size();
  }

  inline int
  mat_type(cv::Mat& m)
  {
    return m.type();
  }
  inline void
  mat_set(cv::Mat& m, bp::object o)
  {
    if (m.empty())
      throw std::logic_error("The matrix is empty, can not deduce type.");
    //use the m.type and implicitly assume that o is of this type
    mat_set(m, o, m.type());
  }

  inline cv::Mat
  mat_mat_star(cv::Mat& m, cv::Mat& m2)
  {
    return m * m2;
  }

  inline cv::Mat
  mat_scalar_star(cv::Mat& m, double s)
  {
    return m * s;
  }

  inline cv::Mat
  mat_scalar_plus(cv::Mat& m, double s)
  {
    return m + cv::Scalar::all(s);
  }

  inline cv::Mat
  mat_scalar_plus2(cv::Mat& m, cv::Scalar s)
  {
    return m + s;
  }

  inline cv::Mat
  mat_scalar_sub(cv::Mat& m, double s)
  {
    return m - cv::Scalar::all(s);
  }

  inline cv::Mat
  mat_scalar_sub2(cv::Mat& m, cv::Scalar s)
  {
    return m - s;
  }

  inline cv::Mat
  mat_scalar_div(cv::Mat& m, double s)
  {
    return m / s;
  }

  inline cv::Mat
  mat_mat_plus(cv::Mat& m, cv::Mat& m2)
  {
    return m + m2;
  }

  inline cv::Mat
  mat_mat_sub(cv::Mat& m, cv::Mat& m2)
  {
    return m - m2;
  }
  inline cv::Mat
  mat_mat_div(cv::Mat& m, cv::Mat& m2)
  {
    return m / m2;
  }

  inline cv::Mat
  roi(cv::Mat& m, cv::Rect region)
  {
    return m(region);
  }
  inline std::string
  tostr(cv::Mat& m)
  {
    std::stringstream ss;
#if CV_MAJOR_VERSION == 3
    ss << cv::Formatter::get(cv::Formatter::FMT_PYTHON)->format(m);
#else
    cv::Formatter::get("python")->write(ss,m);
#endif
    return ss.str();
  }
  inline cv::Mat
  transpose(cv::Mat& m)
  {
    return m.t();
  }

  boost::shared_ptr<cv::Mat> from_numpy( bp::numeric::array array)
  {
    cv::Mat m;
    mat_from_array(m,array);
    return boost::shared_ptr<cv::Mat>(new cv::Mat(m));
  }

}

namespace opencv_wrappers
{
  void
  wrap_mat()
  {
    import_array();
    bp::numeric::array::set_module_and_type("numpy", "ndarray");

    typedef std::vector<uchar> buffer_t;
    bp::class_<std::vector<uchar> >("buffer").def(bp::vector_indexing_suite<std::vector<uchar>, false>());

    bp::class_<cv::_InputArray>("InputArray");
    bp::class_<cv::_OutputArray>("OuputArray");
    bp::implicitly_convertible<cv::Mat, cv::_InputArray>();
    bp::implicitly_convertible<cv::Mat, cv::_OutputArray>();

    //mat definition
    bp::class_<cv::Mat, boost::shared_ptr<cv::Mat> > Mat_("Mat");
    Mat_.def(bp::init<>());
    Mat_.def(bp::init<int, int, int>());
    Mat_.def(bp::init<cv::Size, int>());
    Mat_.def(bp::init<buffer_t>());
    Mat_.def("__init__", bp::make_constructor(from_numpy));
    Mat_.def_readonly("rows", &cv::Mat::rows, "the number of rows");
    Mat_.def_readonly("cols", &cv::Mat::cols, "the number of columns");
    Mat_.def("row", &cv::Mat::row, "get the row at index");
    Mat_.def("col", &cv::Mat::col, "get the column at index");
    Mat_.def("fromarray", mat_from_array, "Set a cv mat from a numpy array.");
    Mat_.def("toarray", mat_to_array, "Create a python array from a cv::Mat");
    Mat_.def("size", mat_size);
    Mat_.def("empty", &cv::Mat::empty);
    Mat_.def("type", mat_type);
    Mat_.def("convertTo", &cv::Mat::convertTo);
    Mat_.def("clone", &cv::Mat::clone);
    Mat_.def("t", &transpose);
    Mat_.def("roi", roi);
    Mat_.def("__str__",tostr);
    Mat_.def("__mul__", mat_mat_star);
    Mat_.def("__mul__", mat_scalar_star);
    Mat_.def("__add__", mat_mat_plus);
    Mat_.def("__add__", mat_scalar_plus);
    Mat_.def("__add__", mat_scalar_plus2);
    Mat_.def("__sub__", mat_mat_sub);
    Mat_.def("__sub__", mat_scalar_sub);
    Mat_.def("__sub__", mat_scalar_sub2);
    Mat_.def("__div__", mat_mat_div);
    Mat_.def("__div__", mat_scalar_div);

  }
}
