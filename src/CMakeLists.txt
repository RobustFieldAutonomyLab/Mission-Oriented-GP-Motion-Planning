add_library(planning ../include/Planning.h
        Planning3DUUV.cpp)
target_link_libraries(planning
        matplot
        extern_gpmp2
        ${Boost_LIBRARIES}
        gtsam)