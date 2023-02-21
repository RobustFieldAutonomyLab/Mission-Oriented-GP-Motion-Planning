find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
include_directories(include)
add_executable(testSTOMP src/stompPlanning.cpp
        src/STOMPHelper.cpp
        src/stomp.cpp
        src/utils.cpp)
target_link_libraries(testSTOMP
        matplot
        gtsam
        ${Boost_LIBRARIES}
        Eigen3::Eigen
        console_bridge
        ${YAML_CPP_LIBRARIES}
        )