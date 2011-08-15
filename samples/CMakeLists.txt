macro(validate_sample pyfile)
  string(REPLACE ";" " " args "${ARGN}")
  add_test(opencv_${pyfile}
    ${CMAKE_CURRENT_SOURCE_DIR}/validate_sample.sh ${CMAKE_BINARY_DIR}/python_path.sh ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}.py ${args}
    )
endmacro()

validate_sample(circles)
validate_sample(calibrator)
validate_sample(dir_reader)
validate_sample(dot_poser)
validate_sample(orb_sample)
validate_sample(vidcap)
