set(gpmp2_subdirs
        dynamics
        gp
        kinematics
        obstacle
        mission)

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