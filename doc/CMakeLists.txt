#add_custom_target(doc
#  COMMENT "Generate all documentation" VERBATIM
#  )
find_sphinx()
if(SPHINX_BUILD)
  set(DOC_SOURCE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/source
    ${ecto_opencv_SOURCE_DIR}/samples
  )
  configure_file(doc_config.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/doc_config.py
  )
  sphinx(${PROJECT_NAME}-html ${CMAKE_CURRENT_SOURCE_DIR}/source  ${CMAKE_CURRENT_BINARY_DIR}/html
    ${CMAKE_BINARY_DIR}/lib
    ${CMAKE_CURRENT_BINARY_DIR}
    )
  #add_dependencies(doc opencv-html)
endif()
