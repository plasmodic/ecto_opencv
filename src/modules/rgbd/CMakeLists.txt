# if (NOT (DEFINED ocv_define_module))
#  return()
# endif()
# 
# merge some RGBD functions into a module
set(the_description "a set of RGBD functions")
ocv_define_module(rgbd)

install(TARGETS opencv_rgbd
        DESTINATION lib
)
