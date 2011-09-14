ectomodule(imgproc
    module.cpp
    Adder.cpp
    BilateralFilter.cpp
    BitWiseAnd.cpp
    BitWiseNot.cpp
    Canny.cpp
    CartToPolar.cpp
    cvtColor.cpp
    GaussianBlur.cpp
    Erode.cpp
    NanRemove.cpp
    Quantize.cpp
    Scale.cpp
    Scharr.cpp
    Sobel.cpp
    Translate.cpp
    )

link_ecto(imgproc ${OpenCV_LIBS})
install_ecto_module(imgproc)