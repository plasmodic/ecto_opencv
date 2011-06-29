add_subdirectory(opencv)
#opencv boost::python bindings
add_subdirectory(cv_bp)
add_subdirectory(pcl)

ectomodule(lazer lazer/lazer.cpp)
link_ecto(lazer ${OpenCV_LIBS})
install_ecto_module(lazer)


ectomodule(line_mod line_mod/line_mod.cpp)
link_ecto(line_mod ${OpenCV_LIBS})
install_ecto_module(line_mod)
