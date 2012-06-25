ectomodule(rgbd module.cpp
                Normal.cpp
                Odometry.cpp
                Plane.cpp
)

add_dependencies(rgbd_ectomodule opencv_rgbd)

link_ecto(rgbd ${OpenCV_LIBS}
               ${OpenCV_LIBRARIES}
               opencv_rgbd
)
install_ecto_module(rgbd)
