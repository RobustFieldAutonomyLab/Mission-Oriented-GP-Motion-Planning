#add_executable(test main.cpp)
#target_link_libraries(test PUBLIC
#        planning
#        matplot
#        Threads::Threads
#        extern_gpmp2
#        ${Boost_LIBRARIES}
#        gtsam)

#add_executable(testMobile2D MobileRobot2DPlanning.cpp)
#target_link_libraries(testMobile2D PUBLIC
#        planning
#        matplot
#        Threads::Threads
#        extern_gpmp2
#        ${Boost_LIBRARIES}
#        gtsam)
#
#add_executable(testPoint3D PointRobot3DPlanning.cpp)
#target_link_libraries(testPoint3D PUBLIC
#        planning
#        matplot
#        Threads::Threads
#        extern_gpmp2
#        ${Boost_LIBRARIES}
#        gtsam)

add_executable(testMobile3D Mobile3DPlanning.cpp)
target_link_libraries(testMobile3D PUBLIC
        planning
        matplot
        Threads::Threads
        extern_gpmp2
        ${YAML_CPP_LIBRARIES}
        ${Boost_LIBRARIES}
        gtsam)

