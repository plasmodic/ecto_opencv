cmake_minimum_required(VERSION 2.8)
project(ecto_opencv)

find_package(catkin REQUIRED ecto opencv_candidate)
if (${catkin_VERSION} VERSION_GREATER "0.5.28")
catkin_package(DEPENDS ecto opencv_candidate)
else()
find_package(ROS REQUIRED ecto opencv_candidate)
catkin_stack()
catkin_project(ecto_opencv DEPENDS ecto opencv_candidate)
set(catkin_LIBRARIES ${ROS_LIBRARIES})
set(catkin_INCLUDE_DIRS ${ROS_INCLUDE_DIRS})
set(CATKIN_DEVEL_PREFIX ${CMAKE_BINARY_DIR})
set(CATKIN_PACKAGE_LIB_DESTINATION lib)
set(CATKIN_PACKAGE_PYTHON_BUILD_DESTINATION gen/py/${PROJECT_NAME}/)
set(CATKIN_PACKAGE_PYTHON_DESTINATION ${INSTALLED_PYTHONPATH}/${PROJECT_NAME}/)
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

option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
  add_subdirectory(samples)
endif()

ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc doc)
