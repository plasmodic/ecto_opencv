project(opencv)

ectomodule(calib calib.cpp)
link_ecto(calib ${OpenCV_LIBS})
install_ecto_module(calib)

ectomodule(highgui highgui.cpp)
link_ecto(highgui ${OpenCV_LIBS})
install_ecto_module(highgui)

ectomodule(imgproc imgproc.cpp)
link_ecto(imgproc ${OpenCV_LIBS})
install_ecto_module(imgproc)

ectomodule(features2d features2d.cpp feature_extraction.cpp)
link_ecto(features2d ${OpenCV_LIBS})
#include_ecto(features2d ./)
install_ecto_module(features2d)
