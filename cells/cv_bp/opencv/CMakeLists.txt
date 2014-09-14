find_package(OpenCV REQUIRED core highgui)
find_package(Boost COMPONENTS
  python
  REQUIRED
  )
  
set(CMAKE_BUILD_TYPE Debug)

find_package(PythonLibs
  REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(numpy)
if(numpy-NOTFOUND)
    message(WARNING "Not building cv_bp as it requires numpy")
    return()
endif()
#detect the python version and install directories
find_package(PythonInterp REQUIRED)

if(NOT PYTHON_PACKAGES_PATH )
  execute_process(COMMAND ${PYTHON_EXECUTABLE} --version
    ERROR_VARIABLE PYTHON_VERSION_FULL
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  
  string(REGEX MATCH "[0-9].[0-9]" PYTHON_VERSION_MAJOR_MINOR "${PYTHON_VERSION_FULL}")
  
  if(UNIX)
    if(APPLE)
      set(PYTHON_PACKAGES_PATH lib/python${PYTHON_VERSION_MAJOR_MINOR}/site-packages 
        CACHE PATH "Where to install the python packages.")
    else() #debian based assumed, install to the dist-packages.
      set(PYTHON_PACKAGES_PATH lib/python${PYTHON_VERSION_MAJOR_MINOR}/dist-packages 
        CACHE PATH "Where to install the python packages.")
    endif()
  elseif(WIN32)
    get_filename_component(PYTHON_PATH "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\${PYTHON_VERSION_MAJOR_MINOR}\\InstallPath]" ABSOLUTE CACHE)
    set(PYTHON_PACKAGES_PATH "${PYTHON_PATH}/Lib/site-packages" 
      CACHE PATH "Where to install the python packages.")
  endif()
  
endif()

include_directories(
  ${PYTHON_INCLUDE_PATH}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_NUMPY_INCLUDE_DIR}
  )

set(bp_opencv_target opencv_boost_python)

add_library(${bp_opencv_target} SHARED
  opencv.cpp
  cv_core.cpp
  cv_mat.cpp
  cv_points.cpp
  cv_highgui.cpp
  highgui_defines.cpp
  )

set_target_properties(${bp_opencv_target}
  PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/
  OUTPUT_NAME cv_bp
  PREFIX ""
  )

target_link_libraries(${bp_opencv_target}
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${OpenCV_LIBS}
  ${cv_backports_LIBRARIES}
  )

install(TARGETS ${bp_opencv_target}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/ COMPONENT main)
