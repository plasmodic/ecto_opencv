cmake_minimum_required(VERSION 2.8)
project(ecto_opencv)
find_package(catkin REQUIRED)
catkin_stack(ecto_opencv)

set(${PROJECT_NAME}_GITHUB_URL https://github.com/plasmodic/ecto_opencv)

if(NOT CMAKE_BUILD_TYPE)#Only do this the on the first run, if the build type hasn't been set prior
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif()

add_definitions(-Wno-strict-aliasing)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV headers: ${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(ecto REQUIRED)
git_status(ecto_opencv)

find_package(Boost COMPONENTS serialization filesystem system REQUIRED)

if(NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/ecto_opencv)

configure_file(python/__init__.py.in
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/__init__.py
  @ONLY)

ecto_python_env_gen(${CMAKE_BINARY_DIR}/lib ${CMAKE_CURRENT_SOURCE_DIR}/python)
set_ecto_install_package_name(ecto_opencv)

install(FILES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/__init__.py
  DESTINATION ${ecto_module_PYTHON_INSTALL} COMPONENT main
  )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(cells)
add_subdirectory(doc)
add_subdirectory(src)

option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
  add_subdirectory(samples)
endif()

catkin_project(ecto_opencv
 INCLUDE_DIRS
 include
 LIBRARIES opencv_candidate
)

catkin_add_env_hooks(10ecto_opencv.buildspace.sh 10ecto_opencv.install.sh)
