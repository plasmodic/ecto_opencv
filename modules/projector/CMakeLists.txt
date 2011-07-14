project(projector)

ectomodule(projector 
  projector.cpp
  PatternProjector.cpp
  calibrator.cpp
  warper.cpp
  fitter.cpp
  )
link_ecto(projector ${OpenCV_LIBS})
install_ecto_module(projector)

