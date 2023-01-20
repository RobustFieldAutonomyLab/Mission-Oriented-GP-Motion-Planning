cmake_minimum_required(VERSION 3.16)
project(GPMP_STR)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS_INIT} -std=c++11 -fPIC")

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)

# Boost - same requirement as gtsam
find_package(Boost 1.55 REQUIRED system serialization filesystem thread date_time timer system program_options)
include_directories(${Boost_INCLUDE_DIR})

#OMPL - for comparing
find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIR})

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_subdirectory(third_party/matplotplusplus)
include_directories(include)
include_directories(third_party/matplotplusplus/source)

set(gpmp2_subdirs
        gpmp2/dynamics
        gpmp2/gp
        gpmp2/kinematics
        gpmp2/obstacle
        gpmp2/mission)

set(gpmp2_srcs)

# files want to be excluded
set(excluded_sources "")

# Library sources
foreach(subdir ${gpmp2_subdirs})
    file(GLOB subdir_srcs "${subdir}/*.cpp" "${subdir}/*.h")
    list(REMOVE_ITEM subdir_srcs "${excluded_sources}")

    list(APPEND gpmp2_srcs ${subdir_srcs})
    message(STATUS "Building Module: ${subdir}")
endforeach(subdir)

add_library(extern_gpmp2 ${gpmp2_srcs})
target_link_libraries(extern_gpmp2
        ${Boost_LIBRARIES}
        gtsam)

add_library(planning include/Planning.h
        src/Planning2D.cpp
        src/Planning3D.cpp
        src/Planning3DUUV.cpp)
target_link_libraries(planning
        matplot
        extern_gpmp2
        ${Boost_LIBRARIES}
        gtsam)

add_executable(test test/main.cpp)
target_link_libraries(test PUBLIC
        planning
        matplot
        Threads::Threads
        extern_gpmp2
        ${Boost_LIBRARIES}
        gtsam)

add_executable(testMobile2D test/MobileRobot2DPlanning.cpp)
target_link_libraries(testMobile2D PUBLIC
        planning
        matplot
        Threads::Threads
        extern_gpmp2
        ${Boost_LIBRARIES}
        gtsam)

add_executable(testPoint3D test/PointRobot3DPlanning.cpp)
target_link_libraries(testPoint3D PUBLIC
        planning
        matplot
        Threads::Threads
        extern_gpmp2
        ${Boost_LIBRARIES}
        gtsam)

add_executable(testMobile3D test/MobileRobot3DPlanning.cpp)
target_link_libraries(testMobile3D PUBLIC
        planning
        matplot
        Threads::Threads
        extern_gpmp2
        ${Boost_LIBRARIES}
        gtsam)

add_executable(testNYC test/Mobile3DNYC.cpp)
target_link_libraries(testNYC PUBLIC
        planning
        matplot
        Threads::Threads
        extern_gpmp2
        ${Boost_LIBRARIES}
        gtsam)

add_library(OMPLPlanning include/OMPLObjective.h
        include/OMPLConstraint.h
        src/OMPLHelper.cpp)
target_link_libraries(OMPLPlanning
        ompl
        matplot
        gtsam
        ${Boost_LIBRARIES})

add_executable(testRRTstar test/RRTstarPlanning.cpp)
target_link_libraries(testRRTstar PUBLIC
        OMPLPlanning
        ompl
        ${Boost_LIBRARIES}
        gtsam
        matplot)

add_executable(testKPiece test/KPiecePlanning.cpp)
target_link_libraries(testKPiece PUBLIC
        OMPLPlanning
        ompl
        ${Boost_LIBRARIES}
        gtsam
        matplot)

#add_executable(testGA test/GAPlanning.cpp)
#target_link_libraries(testKPiece PUBLIC
#        OMPLPlanning
#        ompl
#        ${Boost_LIBRARIES}
#        gtsam
#        matplot)
