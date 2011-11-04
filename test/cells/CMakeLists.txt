ectomodule(opencv_test
  module.cpp
  ImageGen.cpp
  MatGen.cpp
  )
link_ecto(opencv_test ${OpenCV_LIBS})
