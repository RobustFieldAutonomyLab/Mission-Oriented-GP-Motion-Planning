add_library(planning ../include/Planning.h
        Planning2D.cpp
        Planning3D.cpp
        Planning3DUUV.cpp)
target_link_libraries(planning
        matplot
        extern_gpmp2
        ${Boost_LIBRARIES}
        gtsam)