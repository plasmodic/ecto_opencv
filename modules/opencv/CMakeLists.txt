project(opencv)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})#boost

add_subdirectory(calib)
add_subdirectory(features2d)

ectomodule(highgui highgui.cpp)
link_ecto(highgui ${OpenCV_LIBS})
install_ecto_module(highgui)

ectomodule(imgproc imgproc.cpp)
link_ecto(imgproc ${OpenCV_LIBS})
install_ecto_module(imgproc)

