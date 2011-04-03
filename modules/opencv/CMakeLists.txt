project(opencv)
ectomodule(calib calib.cpp)
link_ecto(calib ${OpenCV_LIBS})
ectomodule(imageproc imageproc.cpp)
link_ecto(imageproc ${OpenCV_LIBS})

add_subdirectory(wrappings)

