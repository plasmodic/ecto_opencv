project(projector)

ectomodule(projector 
  projector.cpp
  PatternProjector.cpp
  )
link_ecto(projector ${OpenCV_LIBS})
install_ecto_module(projector)

