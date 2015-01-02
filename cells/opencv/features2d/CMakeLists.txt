ectomodule(features2d DESTINATION ecto_opencv/ecto_cells
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
  )

find_package(OpenCV REQUIRED)
link_ecto(features2d ${catkin_LIBRARIES}
                     ${OpenCV_LIBRARIES}
)

#For Hamming distance
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-msse4.2 GCC_HAS_SSE)
if(GCC_HAS_SSE)
  set_target_properties(features2d_ectomodule PROPERTIES COMPILE_FLAGS "-msse4.2")
endif()
