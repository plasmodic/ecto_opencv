ectomodule(highgui
    module.cpp
    FPSDrawer.cpp
    ImageReader.cpp
    ImageSaver.cpp
    imread.cpp
    imshow.cpp
    OpenNICapture.cpp
    VideoCapture.cpp
    VideoWriter.cpp
    )
link_ecto(highgui ${OpenCV_LIBS})
install_ecto_module(highgui)