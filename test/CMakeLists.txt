macro(pytest pyfile)
    string(REPLACE ";" " " args "${ARGN}")
    add_test(${pyfile}
        ${CMAKE_CURRENT_SOURCE_DIR}/run_test.sh ${CMAKE_BINARY_DIR}/python_path.sh ${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}.py ${args}
        )
endmacro()

macro(data_download VAR DATA_FILE)
  set(data_base_url http://vault.willowgarage.com/wgdata1/vol1/ecto_data)
  set(${VAR} ${PROJECT_BINARY_DIR}/data/${DATA_FILE})
  file(GLOB ${DATA_FILE}_exists ${${VAR}})
  if(NOT EXISTS ${${VAR}})
    message(STATUS "Data fetch.\n** Downloading:\n** ${data_base_url}/${DATA_FILE}\n** to:\n** ${${VAR}}")
    file(DOWNLOAD ${data_base_url}/${DATA_FILE} ${${VAR}})
  endif()
endmacro()

data_download(T01 t01.bag)
pytest(test_bag ${T01} )
