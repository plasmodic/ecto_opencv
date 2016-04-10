ectomodule(features2d DESTINATION ecto_opencv/ecto_cells
                      INSTALL
  module.cpp
  DescriptorExtractor.cpp
  DrawKeypoints.cpp
  FeatureDetector.cpp
  KeypointsToMat.cpp
  LshMatcher.cpp
  MatchesToMat.cpp
  match_draw.cpp
  matcher.cpp
  ORB.cpp
  ransac_est.cpp
  )

find_package(OpenCV REQUIRED)
link_ecto(features2d ${catkin_LIBRARIES}
                     ${OpenCV_LIBRARIES}
)
