add_library(opencv_python SHARED
  opencv.cpp
  cv_core.cpp
  cv_mat.cpp
  cv_points.cpp
  cv_highgui.cpp
  )

set_target_properties(opencv_python
  PROPERTIES
  OUTPUT_NAME bindings
  PREFIX ""
  )

target_link_libraries(opencv_python
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${OpenCV_LIBS}
  )
