cmake_minimum_required(VERSION 3.16)
project(GPMP_STR)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS_INIT} -std=c++11 -fPIC")

set (USE_BENCHMARK ON)

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)

# Boost - same requirement as gtsam
find_package(Boost 1.65 REQUIRED system serialization filesystem thread date_time timer system program_options)
include_directories(${Boost_INCLUDE_DIR})

if(${USE_BENCHMARK})
    #OMPL - for comparing
    find_package(ompl REQUIRED)
    include_directories(${OMPL_INCLUDE_DIR})
 endif()


set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})

add_subdirectory(third_party/matplotplusplus)
include_directories(third_party/matplotplusplus/source)

include_directories(include)

add_subdirectory(gpmp2)

add_subdirectory(src)
add_subdirectory(tests)

if(${USE_BENCHMARK})
    add_subdirectory(benchmark/ompl)
    add_subdirectory(benchmark/stomp)
endif()
