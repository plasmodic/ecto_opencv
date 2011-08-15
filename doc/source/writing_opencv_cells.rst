writing opencv cells
=====================

Since opencv uses ``cv::Mat`` which behaves like a smart pointer, it is
important that cells not munge the output data by working in place.
