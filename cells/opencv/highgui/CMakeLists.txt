find_library(V4L2_LIBRARY v4l2)

if (V4L2_LIBRARY)
  message(STATUS "Adding support for v4l2 image capture")
  set(EXTRA_SRC V4LCapture.cpp)
  set(EXTRA_LIB v4l2)
endif()

ectomodule(highgui DESTINATION ecto_opencv
                   INSTALL
    module.cpp
    FPSDrawer.cpp
    DoubleDrawer.cpp
    FileIO.cpp
    ImageReader.cpp
    ImageSaver.cpp
    imread.cpp
    imshow.cpp
    MatPrinter.cpp
    NiConverters.cpp
    #OpenNICapture.cpp
    ${EXTRA_SRC}
    VideoCapture.cpp
    VideoWriter.cpp
    serialization.cpp
    )
link_ecto(highgui ${OpenCV_LIBS} ${EXTRA_LIB} ${cv_backports_LIBRARIES})
