
ectomodule(calib
  module.cpp
  calib.cpp 
  CameraIntrinsics.cpp
  PlanarSegmentation.cpp 
  PatternDrawer.cpp
  PatternDetector.cpp
  CircleDrawer.cpp
  SubrectRectifier.cpp
  DepthTo3d.cpp
  pose_drawing.cpp
  Latch.cpp
  PlaneFitter.cpp
  TransformCompose.cpp
  FiducialPoseEst.cpp
  CameraCalibrator.cpp
  GatherPoints.cpp
  DepthMask.cpp
  PointsTo3d.cpp
  )
add_dependencies(calib_ectomodule opencv_rgbd)
link_ecto(calib
          ${OpenCV_LIBS}
          ${OpenCV_LIBRARIES}
          opencv_rgbd
)
install_ecto_module(calib)
