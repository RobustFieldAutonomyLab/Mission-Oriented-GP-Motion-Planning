add_executable(testGA src/GAPlanning.cpp src/GeneticAlgorithm.cpp)
target_link_libraries(testGA PUBLIC
        ${Boost_LIBRARIES}
        gtsam
        matplot)