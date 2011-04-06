add_subdirectory(opencv)

ectomodule(lazer lazer/lazer.cpp)
link_ecto(lazer ${OpenCV_LIBS})

ectomodule(orb ORB/orb.cpp ORB/FASTHarris.cpp)
link_ecto(orb ${OpenCV_LIBS})