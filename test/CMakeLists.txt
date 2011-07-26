macro(pytest pyfile)
    string(REPLACE ";" " " args "${ARGN}")
    add_test(${pyfile}
        /usr/bin/env
        PYTHONPATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}:${PROJECT_SOURCE_DIR}/python:$PYTHONPATH
        ${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}.py ${args}
        )
endmacro()

set(data_base_url http://vault.willowgarage.com/wgdata1/vol1/ecto_data)
macro(data_download VAR DATA_FILE)
  set(${VAR} ${PROJECT_BINARY_DIR}/data/${DATA_FILE})
  file(GLOB ${DATA_FILE}_exists ${${VAR}})
  if("" STREQUAL "${${DATA_FILE}_exists}")
    message(STATUS "Data fetch.\n** Downloading:\n** ${data_base_url}/${DATA_FILE}\n** to:\n** ${${VAR}}")
    file(DOWNLOAD ${data_base_url}/${DATA_FILE} ${${VAR}})
  endif()
endmacro()

data_download(T01 t01.bag)
pytest(${file} ${T01} )
