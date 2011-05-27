add_subdirectory(opencv)

ectomodule(lazer lazer/lazer.cpp)
link_ecto(lazer ${OpenCV_LIBS})

ectomodule(line_mod line_mod/line_mod.cpp)
link_ecto(line_mod ${OpenCV_LIBS})
