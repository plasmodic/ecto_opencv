# merge some RGBD functions into a module
#set(the_description "a set of RGBD functions")
#ocv_define_module(rgbd)

find_package(OpenCV REQUIRED)

add_library(opencv_rgbd src/compute_normals.cpp
                        src/depth_to_3d.cpp
                        src/utils.cpp
)

target_link_libraries(opencv_rgbd ${OpenCV_LIBS}
                                  ${OpenCV_LIBRARIES}
)

install(TARGETS opencv_rgbd
        DESTINATION lib
)

install(DIRECTORY include/opencv2/rgbd
        DESTINATION include/opencv2
        COMPONENT main
)

# Add some tests
find_package(GTest QUIET)

include_directories(${GTEST_INCLUDE_DIRS})

add_executable(rgbd_tests test/test_main.cpp
                                    test/test_compute_normals.cpp
                                    test/test_precomp.cpp
)
target_link_libraries(rgbd_tests ${GTEST_BOTH_LIBRARIES}
                                 ${OpenCV_LIBRARIES}
                                 opencv_rgbd
)

add_test(rgbd_tests rgbd_tests)
