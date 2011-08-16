project(opencv)
if("${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}" VERSION_LESS "1.42")
    message(STATUS "Boost version: ${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION} using copy of property_tree.")
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})#boost property tree.
endif()
add_subdirectory(calib)
add_subdirectory(features2d)
add_subdirectory(imgproc)
add_subdirectory(highgui)
