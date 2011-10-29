ectomodule(features2d
  module.cpp
  features2d.cpp
  FAST.cpp
  matcher.cpp
  match_draw.cpp
  ransac_est.cpp
  ORB.cpp
  )
link_ecto(features2d ${OpenCV_LIBS})
install_ecto_module(features2d)
set_target_properties(features2d_ectomodule PROPERTIES COMPILE_FLAGS "-msse4.2")
