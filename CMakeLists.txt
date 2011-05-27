cmake_minimum_required(VERSION 2.8)
project(ecto_opencv)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-strict-aliasing -Wall -Wl,--no-undefined")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DBOOST_DISABLE_ASSERTS")

#set the default path for built libraries to the "ecto_opencv" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/ecto_opencv)

configure_file(python/__init__.py.in 
  ${LIBRARY_OUTPUT_PATH}/__init__.py
  @ONLY)

#use the cmake cache, to -DOpenCV_dir=<path_to_build_of_opencv>
#  svn co https://code.ros.org/svn/opencv/trunk
#  and cmake that baby
find_package(OpenCV REQUIRED)
find_package(ecto REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)

add_subdirectory(modules)



