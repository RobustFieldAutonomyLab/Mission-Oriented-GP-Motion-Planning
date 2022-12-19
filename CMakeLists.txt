cmake_minimum_required(VERSION 3.16)
project(GPMP_STR)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)
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

set(gpmp2_subdirs
        gpmp2/dynamics gpmp2/gp gpmp2/kinematics gpmp2/obstacle)

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

# add_subdirectory(extern/pybind11)
# pybind11_add_module(GPPlanning src/wrapper.cpp
#         src/Planning2D.cpp
#         src/Planning3D.cpp)
# target_link_libraries(GPPlanning PRIVATE
#         pybind11::module
#         extern_gpmp2
#         ${Boost_LIBRARIES}
#         gtsam)

add_library(planning include/Planning.h
        src/Planning2D.cpp
        src/Planning3D.cpp
        src/Planning3DUUV.cpp)
target_link_libraries(planning
        matplot
        extern_gpmp2
        ${Boost_LIBRARIES}
        gtsam)

add_executable(test src/main.cpp)
target_link_libraries(test
        planning
        matplot
        extern_gpmp2
        ${Boost_LIBRARIES}
        gtsam)

add_executable(testPoint2D src/PointRobot2DPlanning.cpp)
target_link_libraries(testPoint2D PUBLIC
        matplot
        Threads::Threads
        extern_gpmp2
        ${Boost_LIBRARIES}
        gtsam)

add_executable(testPoint3D src/PointRobot3DPlanning.cpp)
target_link_libraries(testPoint3D PUBLIC
        planning
        matplot
        Threads::Threads
        extern_gpmp2
        ${Boost_LIBRARIES}
        gtsam)

add_executable(testMobile3D src/MobileRobot3DPlanning.cpp)
target_link_libraries(testMobile3D PUBLIC
        planning
        matplot
        Threads::Threads
        extern_gpmp2
        ${Boost_LIBRARIES}
        gtsam)
