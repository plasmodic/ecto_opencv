ectomodule(imgproc DESTINATION ecto_opencv
                   INSTALL
    module.cpp
    Adder.cpp
    BilateralFilter.cpp
    BitWiseAnd.cpp
    BitWiseNot.cpp
    Canny.cpp
    CartToPolar.cpp
    EqualizeHistogram.cpp
    Erode.cpp
    cvtColor.cpp
    GaussianBlur.cpp
    MedianBlur.cpp
    NanRemove.cpp
    Quantize.cpp
    Scale.cpp
    Scharr.cpp
    Sobel.cpp
    Split.cpp
    SplitThree.cpp
    Subtract.cpp
    VerticalStepFilter.cpp
    )

link_ecto(imgproc ${OpenCV_LIBS})
