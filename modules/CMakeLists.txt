add_subdirectory(opencv)
#opencv boost::python bindings
add_subdirectory(cv_bp)
add_subdirectory(pcl)
add_subdirectory(tod)


ectomodule(lazer lazer/lazer.cpp)
link_ecto(lazer ${OpenCV_LIBS})

ectomodule(line_mod line_mod/line_mod.cpp)
link_ecto(line_mod ${OpenCV_LIBS})

