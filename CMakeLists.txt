cmake_minimum_required(VERSION 2.8)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "The build type.")
project(ecto_opencv)

add_definitions("-fno-strict-aliasing -Wall -Wl,--no-undefined")

#set the default path for built libraries to the "ecto_opencv" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/ecto_opencv)

configure_file(python/__init__.py.in 
  ${LIBRARY_OUTPUT_PATH}/__init__.py
  @ONLY)


#use the cmake cache, to -DOpenCV_dir=<path_to_build_of_opencv>
#  svn co https://code.ros.org/svn/opencv/trunk
#  and cmake that baby
find_package(OpenCV 2.3.1 EXACT REQUIRED)
find_package(ecto 0.1.0 REQUIRED)
find_package(Boost COMPONENTS system filesystem serialization REQUIRED)

ecto_python_env_gen(${CMAKE_BINARY_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}/python)
set_ecto_install_package_name(ecto_opencv)

install(FILES ${LIBRARY_OUTPUT_PATH}/__init__.py
  DESTINATION ${ecto_module_PYTHON_INSTALL} COMPONENT main
  )

add_subdirectory(modules)

