include_directories(include)
add_library(OMPLPlanning include/OMPLObjective.h
        include/OMPLConstraint.h
        src/OMPLHelper.cpp)
target_link_libraries(OMPLPlanning
        ompl
        matplot
        gtsam
        ${Boost_LIBRARIES})

add_executable(testRRTstar src/RRTstarPlanning.cpp)
target_link_libraries(testRRTstar PUBLIC
        OMPLPlanning
        ompl
        ${Boost_LIBRARIES}
        gtsam
        matplot)

add_executable(testPRM src/PRMPlanning.cpp)
target_link_libraries(testPRM PUBLIC
        OMPLPlanning
        ompl
        ${Boost_LIBRARIES}
        gtsam
        matplot)


