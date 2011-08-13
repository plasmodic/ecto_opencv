project(projector)

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
link_ecto(projector ${OpenCV_LIBS})
install_ecto_module(projector)

