add_subdirectory(opencv)
ectomodule(lazer lazer/lazer.cpp)
link_ecto(lazer ${OpenCV_LIBS})