include_directories(include)
add_library(OMPLPlanning include/OMPLObjective.h
        include/OMPLConstraint.h
        src/OMPLHelper.cpp)
target_link_libraries(OMPLPlanning
        ompl
        matplot
        gtsam
        ${YAML_CPP_LIBRARIES}
        ${Boost_LIBRARIES})

add_executable(testRRTstar src/RRTstarPlanning.cpp)
target_link_libraries(testRRTstar PUBLIC
        OMPLPlanning
        ompl
        ${YAML_CPP_LIBRARIES}
        ${Boost_LIBRARIES}
        gtsam
        matplot)

