ectomodule(highgui
    module.cpp
    FPSDrawer.cpp
    ImageReader.cpp
    ImageSaver.cpp
    imread.cpp
    imshow.cpp
    OpenNICapture.cpp
    VideoCapture.cpp
    )
link_ecto(highgui ${OpenCV_LIBS})
install_ecto_module(highgui)