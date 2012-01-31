macro(validate_sample pyfile)
  string(REPLACE ";" " " args "${ARGN}")
  add_test(validate_opencv_${pyfile}
    ${CATKIN_ENV} ${CMAKE_CURRENT_SOURCE_DIR}/validate_sample.sh ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}.py ${args}
    )
endmacro()

validate_sample(vidcap)
validate_sample(image_saving)
validate_sample(dir_reader)
validate_sample(bilateral)
validate_sample(calibrator)
validate_sample(dot_poser)
validate_sample(orb_sample)
validate_sample(rescale)

macro(desktop_test_sample pyfile)
  string(REPLACE ";" " " args "${ARGN}")
  add_test(desktop_test_opencv_${pyfile}
    ${CATKIN_ENV} ${CMAKE_CURRENT_SOURCE_DIR}/desktop_test.sh ${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}.py ${args}
    )
endmacro()

option(DESKTOP_TEST "Desktop tests will attempt to run each opencv test which may required a video device." OFF)
if(DESKTOP_TEST)
    desktop_test_sample(vidcap)
    desktop_test_sample(image_saving)
    desktop_test_sample(dir_reader)
    desktop_test_sample(bilateral)
    desktop_test_sample(calibrator)
    desktop_test_sample(dot_poser)
    desktop_test_sample(orb_sample)
    desktop_test_sample(rescale)
endif()
