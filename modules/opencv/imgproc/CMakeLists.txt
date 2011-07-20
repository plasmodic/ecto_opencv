ectomodule(imgproc
    module.cpp
    imgproc.cpp
    cvtColor.cpp
    Sobel.cpp
    )

link_ecto(imgproc ${OpenCV_LIBS})
install_ecto_module(imgproc)