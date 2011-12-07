ectomodule(features2d
  module.cpp
  DescriptorExtractor.cpp
  DrawKeypoints.cpp
  FeatureDetector.cpp
  interfaces.cpp
  KeypointsToMat.cpp
  match_draw.cpp
  matcher.cpp
  ORB.cpp
  ransac_est.cpp
  SIFT.cpp
  )

link_ecto(features2d ${OpenCV_LIBS})
install_ecto_module(features2d)
#For Hamming distance
set_target_properties(features2d_ectomodule PROPERTIES COMPILE_FLAGS "-msse4.2")
