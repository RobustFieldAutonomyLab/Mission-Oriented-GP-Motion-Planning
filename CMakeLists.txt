cmake_minimum_required(VERSION 3.16)
project(GPMP_STR)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS_INIT} -std=c++14 -fPIC")

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)

#find_package(GTSAMCMakeTools)
#include(GtsamMakeConfigFile)
#include(GtsamBuildTypes)
#include(GtsamTesting)

# for unittest scripts
#set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# Boost - same requirement as gtsam
find_package(Boost 1.55 REQUIRED system serialization filesystem thread date_time timer)
include_directories(${Boost_INCLUDE_DIR}
                    gpmp2)

# Process source subdirs
#add_subdirectory(gpmp2)


# Wrapping to MATLAB
#if(EXAMPLES_BUILD_MATLAB_TOOLBOX)
#    # wrap
#    include(GtsamMatlabWrap)
#    wrap_and_install_library(gtsamexamples.h ${PROJECT_NAME} "${CMAKE_CURRENT_SOURCE_DIR}" "")
#endif()

# Install config and export files
#GtsamMakeConfigFile(gtsamexamples)
#export(TARGETS ${gtsamexamples_EXPORTED_TARGETS} FILE gtsamexamples-exports.cmake)



add_library(gpmp2 gpmp2/dynamics/VehicleDynamicsFactorPose2.h
        gpmp2/gp/GPutils.cpp
        gpmp2/gp/GaussianProcessPriorLinear.h
        gpmp2/gp/GaussianProcessInterpolatorLinear.h
        gpmp2/gp/GaussianProcessPriorPose2.h
        gpmp2/kinematics/PointRobotModel.h
        gpmp2/kinematics/PointRobot.cpp
        gpmp2/kinematics/RobotModel.h
#        gpmp2/kinematics/RobotModel-inl.h
        gpmp2/kinematics/ForwardKinematics.h
#        gpmp2/kinematics/ForwardKinematics-inl.h
        gpmp2/kinematics/Pose2MobileBase.cpp
        gpmp2/kinematics/mobileBaseUtils.cpp
        gpmp2/obstacle/ObstaclePlanarSDFFactorGP.h
        gpmp2/obstacle/ObstaclePlanarSDFFactorGPPointRobot.h
        gpmp2/obstacle/PlanarSDF.h
        gpmp2/obstacle/SDFexception.h
#        gpmp2/obstacle/ObstaclePlanarSDFFactorGP-inl.h
        gpmp2/obstacle/ObstacleCost.h
        gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileBase.h
        gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileBase.h
        gpmp2/obstacle/SignedDistanceField.cpp)

target_link_libraries(gpmp2
        ${Boost_LIBRARIES}
        gtsam)

add_subdirectory(extern/pybind11)
pybind11_add_module(GPPlanning main.cpp Planning2D.cpp)
target_link_libraries(GPPlanning PRIVATE
        pybind11::module
        gpmp2
        ${Boost_LIBRARIES}
        gtsam)
#target_compile_definitions(GPPlanning
#        PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO}
#        gpmp2
#        ${Boost_LIBRARIES}
#        gtsam)

#add_executable(GPMP_STR main.cpp Planning2D.cpp)

