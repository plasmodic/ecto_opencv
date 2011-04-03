cmake_minimum_required(VERSION 2.8)
project(ecto_opencv)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

#use the cmake cache, to -DOpenCV_dir=<path_to_build_of_opencv>
#  svn co https://code.ros.org/svn/opencv/trunk
#  and cmake that baby
find_package(OpenCV REQUIRED)
find_package(ecto REQUIRED)

add_subdirectory(modules)


