
ectomodule(calib module.cpp
  calib.cpp 
  PlanarSegmentation.cpp 
  )
link_ecto(calib ${OpenCV_LIBS})
install_ecto_module(calib)

