cmake_minimum_required(VERSION 2.8)
project(ecto_opencv)
message(STATUS "--- ${PROJECT_NAME} ---")

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "The build type.")
add_definitions("-Werror -fno-strict-aliasing -Wall -Wl,--no-undefined")

#use the cmake cache, to -DOpenCV_dir=<path_to_build_of_opencv>
#  svn co https://code.ros.org/svn/opencv/trunk
#  and cmake that baby
find_package(OpenCV 2.3.1 EXACT REQUIRED)
find_package(ecto 0.1.0 REQUIRED)
find_package(Boost COMPONENTS system filesystem serialization REQUIRED)

if(NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib})
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/ecto_opencv)

configure_file(python/__init__.py.in 
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/__init__.py
  @ONLY)
  
ecto_python_env_gen(${CMAKE_BINARY_DIR}/lib ${CMAKE_CURRENT_SOURCE_DIR}/python)
set_ecto_install_package_name(ecto_opencv)

install(FILES ${LIBRARY_OUTPUT_PATH}/__init__.py
  DESTINATION ${ecto_module_PYTHON_INSTALL} COMPONENT main
  )

add_subdirectory(modules)
option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()
