
ectomodule(calib module.cpp
  calib.cpp 
  CameraIntrinsics.cpp
  PlanarSegmentation.cpp 
  PatternDrawer.cpp
  PatternDetector.cpp
  CircleDrawer.cpp
  SubrectRectifier.cpp
  DepthTo3d.cpp
  )
link_ecto(calib ${OpenCV_LIBS})
install_ecto_module(calib)

