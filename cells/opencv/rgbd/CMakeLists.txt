ectomodule(rgbd DESTINATION ecto_opencv
                INSTALL
                Cleaner.cpp
                ClusterDrawer.cpp
                DepthSwapper.cpp
                module.cpp
                Normal.cpp
                Odometry.cpp
                OnPlaneClusterer.cpp
                Plane.cpp
)

link_ecto(rgbd ${catkin_LIBRARIES}
               ${OpenCV_LIBRARIES}
)
