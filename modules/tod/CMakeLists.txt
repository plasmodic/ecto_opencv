project(tod)

find_package(Eigen)

if(Eigen_NOT_FOUND)
return()
endif()

include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

ectomodule(tod tod.cpp)
link_ecto(tod ${OpenCV_LIBS})