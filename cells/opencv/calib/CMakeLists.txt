
ectomodule(calib DESTINATION ecto_opencv
                 INSTALL
  module.cpp
  calib.cpp 
  CameraIntrinsics.cpp
  PatternDrawer.cpp
  PatternDetector.cpp
  CircleDrawer.cpp
  SubrectRectifier.cpp
  DepthTo3d.cpp
  pose_drawing.cpp
  KConverter.cpp
  Latch.cpp
  TransformCompose.cpp
  FiducialPoseEst.cpp
  CameraCalibrator.cpp
  GatherPoints.cpp
  DepthMask.cpp
  PointsTo3d.cpp
  CropBox.cpp
  )

link_ecto(calib ${catkin_LIBRARIES}
                ${opencv_candidate_LIBRARIES}
                ${OpenCV_LIBRARIES}
)
