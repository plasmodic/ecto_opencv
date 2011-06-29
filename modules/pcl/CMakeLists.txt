project(pcl)

#http://www.pointclouds.org/downloads/linux.html
return()
find_package(PCL)

if(NOT PCL_FOUND)
return()
endif()

find_package(Boost COMPONENTS signals thread)

message(STATUS "pcl includes :\n    ${PCL_INCLUDE_DIRS}")
message(STATUS "pcl library  :\n    ${PCL_LIBRARIES}")

include_directories(${PCL_INCLUDE_DIRS})

ectomodule(pcl pcl.cpp)
link_ecto(pcl ${PCL_LIBRARIES})
install_ecto_module(pcl)
