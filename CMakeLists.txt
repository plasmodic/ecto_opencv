cmake_minimum_required(VERSION 2.8)
project(ecto_opencv)

find_package(cv_backports QUIET)
if(cv_backports_FOUND)
find_package(catkin REQUIRED ecto opencv_candidate cv_backports)
catkin_package(DEPENDS ecto opencv_candidate cv_backports)
else()
find_package(catkin REQUIRED ecto opencv_candidate)
catkin_package(DEPENDS ecto opencv_candidate)
endif()

if(NOT CMAKE_BUILD_TYPE)#Only do this the on the first run, if the build type hasn't been set prior
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif()

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV headers: ${OpenCV_INCLUDE_DIRS}")
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

find_package(Boost COMPONENTS serialization filesystem system REQUIRED)

if(NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

# configure and install Python
catkin_python_setup()

add_subdirectory(cells)
add_subdirectory(python)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
  add_subdirectory(samples)
endif()

ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc doc)
