cmake_minimum_required(VERSION 3.16)
project(GPMP_STR)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS_INIT} -std=c++14 -fPIC")

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)

# Boost - same requirement as gtsam
find_package(Boost 1.55 REQUIRED system serialization filesystem thread date_time timer)
include_directories(${Boost_INCLUDE_DIR})

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_subdirectory(third_party/matplotplusplus)
include_directories(include)
include_directories(third_party/matplotplusplus/source)

add_library(extern_gpmp2 gpmp2/dynamics/VehicleDynamics.h
        gpmp2/dynamics/VehicleDynamicsFactorPose2.h
        gpmp2/gp/GPutils.cpp
        gpmp2/gp/GaussianProcessInterpolatorLie.h
        gpmp2/gp/GaussianProcessPriorLie.h
        gpmp2/gp/GaussianProcessPriorLinear.h
        gpmp2/gp/GaussianProcessInterpolatorLinear.h
        gpmp2/gp/GaussianProcessInterpolatorPose2.h
        gpmp2/gp/GaussianProcessPriorPose2.h
        gpmp2/kinematics/PointRobotModel.h
        gpmp2/kinematics/PointRobot.cpp
        gpmp2/kinematics/RobotModel.h
#        gpmp2/kinematics/RobotModel-inl.h
        gpmp2/kinematics/ForwardKinematics.h
        gpmp2/kinematics/Pose2MobileBaseModel.h
#        gpmp2/kinematics/ForwardKinematics-inl.h
        gpmp2/kinematics/Pose2MobileBase.cpp
        gpmp2/kinematics/mobileBaseUtils.cpp
        gpmp2/obstacle/ObstaclePlanarSDFFactorGP.h
        gpmp2/obstacle/ObstaclePlanarSDFFactorGPPointRobot.h
        gpmp2/obstacle/PlanarSDF.h
        gpmp2/obstacle/ObstacleSDFFactor.h
        gpmp2/obstacle/SDFexception.h
#        gpmp2/obstacle/ObstaclePlanarSDFFactorGP-inl.h
        gpmp2/obstacle/ObstacleCost.h
        gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileBase.h
        gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileBase.h
        gpmp2/obstacle/SignedDistanceField.cpp
        gpmp2/obstacle/ObstacleSDFFactor.h
        gpmp2/obstacle/ObstacleSDFFactorGP.h
        gpmp2/kinematics/Pose3MobileBase.cpp
        gpmp2/kinematics/Pose3MobileBaseModel.h
        gpmp2/dynamics/VehicleDynamicsFactorPose3.h)

target_link_libraries(extern_gpmp2
        ${Boost_LIBRARIES}
        gtsam)

# add_subdirectory(extern/pybind11)
# pybind11_add_module(GPPlanning src/wrapper.cpp
#         src/Planning2D.cpp
#         src/Planning3D.cpp)
# target_link_libraries(GPPlanning PRIVATE
#         pybind11::module
#         extern_gpmp2
#         ${Boost_LIBRARIES}
#         gtsam)

add_executable(test src/main.cpp
        src/Planning2D.cpp
        src/Planning3D.cpp)
target_link_libraries(test
        matplot
        extern_gpmp2
        ${Boost_LIBRARIES}
        gtsam)

add_executable(testPoint2D src/PointRobot2DPlanning.cpp)
target_link_libraries(testPoint2D PUBLIC
        matplot
        Threads::Threads
        extern_gpmp2
#        GPMP2
        ${Boost_LIBRARIES}
        gtsam)

add_executable(testPoint3D
        src/PointRobot3DPlanning.cpp
        src/Planning3D.cpp)
target_link_libraries(testPoint3D PUBLIC
        matplot
        Threads::Threads
        extern_gpmp2
#        GPMP2
        ${Boost_LIBRARIES}
        gtsam)



