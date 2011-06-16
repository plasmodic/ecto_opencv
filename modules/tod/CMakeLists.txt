project(tod)

find_package(Eigen)

if(Eigen_NOT_FOUND)
return()
endif()

include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

ectomodule(tod tod.cpp)
link_ecto(tod ${OpenCV_LIBS})


find_package(objcog_db)
if(NOT objcog_db_FOUND)
return()
endif()


ectomodule(objcog_db objcog_db.cpp)
link_ecto(objcog_db ${OpenCV_LIBS} ${objcog_db_LIBRARIES})