ectomodule(opencv_test DESTINATION ecto_opencv
  module.cpp
  ImageGen.cpp
  MatGen.cpp
  )
link_ecto(opencv_test ${OpenCV_LIBS})
