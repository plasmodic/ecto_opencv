ectomodule(imgproc
    module.cpp
    Adder.cpp
    BitWiseAnd.cpp
    BitWiseNot.cpp
    CartToPolar.cpp
    cvtColor.cpp
    GaussianBlur.cpp
    Scale.cpp
    Scharr.cpp
    Sobel.cpp
    Translate.cpp
    )

link_ecto(imgproc ${OpenCV_LIBS})
install_ecto_module(imgproc)