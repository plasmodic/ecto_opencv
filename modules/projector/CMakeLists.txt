project(projector)

find_ros_package(pcl)
include_directories(${pcl_INCLUDE_DIRS})

ectomodule(projector 
  projector.cpp
  PatternProjector.cpp
  calibrator.cpp
  fitter.cpp
  depth_warper.cpp
  fiducial_warper.cpp
  Buttons.cpp
  InteractionBox.cpp
  )
link_ecto(projector ${OpenCV_LIBS} ${pcl_LIBRARIES})
install_ecto_module(projector)

