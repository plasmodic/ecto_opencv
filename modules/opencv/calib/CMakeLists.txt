
ectomodule(calib module.cpp
  calib.cpp 
  CameraIntrinsics.cpp
  PlanarSegmentation.cpp 
  PatternDrawer.cpp
  PatternDetector.cpp
  CircleDrawer.cpp
  SubrectRectifier.cpp
  DepthTo3d.cpp
  pose_drawing.cpp
  impl/depth_to_3d.cpp
  Latch.cpp
  PlaneFitter.cpp
  TransformCompose.cpp
  )
link_ecto(calib ${OpenCV_LIBS})
install_ecto_module(calib)

