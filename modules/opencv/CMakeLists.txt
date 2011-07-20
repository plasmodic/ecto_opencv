project(opencv)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})#boost
add_subdirectory(calib)
add_subdirectory(features2d)
add_subdirectory(imgproc)
add_subdirectory(highgui)
