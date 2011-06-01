ecto_opencv
========================================

ecto_opencv is a collection of ecto modules and tools that make
ones life much more rosey when developing perception based ecto
graphs.

ecto_opencv.cv_bp
----------------------------------------

Some boost::python bindings for opencv, so that some values may be inspected
from python. This will also contain some conversions to and from opencv's
c_types based python wrappers.


Dependencies
----------------------------------------

 - cmake
 - git
 - python
 - boost
 - ecto https://github.com/plasmodic/ecto
      
      If you prefer not to build things, a rolling daily build of
      trunk is available via apt-get::
      
        sudo apt-add-repository ppa:ethan-rublee/ppa
        sudo apt-get update
        sudo apt-get install libecto-dev
      
 - opencv trunk https://code.ros.org/svn/opencv/trunk/opencv
      
      If you prefer not to build things, a rolling daily build of
      trunk is available via apt-get. You can grab it from the same
      place (ppa:ethan-rublee/ppa)::
      
        sudo apt-get install libopencv-dev
        
 - pcl 1.0 (non ros version, standalone)
      
      http://www.pointclouds.org/downloads/linux.html
      
      Quoted here for convenience::
        
        sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
        sudo apt-get update
        sudo apt-get install libpcl-dev

