ectomodule(features2d DESTINATION ecto_opencv
                      INSTALL
  module.cpp
  DescriptorExtractor.cpp
  DrawKeypoints.cpp
  FeatureDetector.cpp
  interfaces.cpp
  KeypointsToMat.cpp
  LshMatcher.cpp
  MatchesToMat.cpp
  match_draw.cpp
  matcher.cpp
  ORB.cpp
  ransac_est.cpp
  SIFT.cpp
  )

find_package(OpenCV REQUIRED)
link_ecto(features2d
    ${OpenCV_LIBS}
    opencv_candidate)

#For Hamming distance
set_target_properties(features2d_ectomodule PROPERTIES COMPILE_FLAGS "-msse4.2")
