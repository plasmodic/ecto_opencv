project(tod)

find_package(Eigen REQUIRED)

message(STATUS "Eigen3 includes :\n    ${EIGEN_INCLUDE_DIRS}")
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
ectomodule(tod Camera.cpp PoseRT.cpp)
link_ecto(tod ${OpenCV_LIBS})