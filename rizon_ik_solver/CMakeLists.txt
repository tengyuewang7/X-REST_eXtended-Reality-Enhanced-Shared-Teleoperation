cmake_minimum_required(VERSION 3.10)
project(rizon_ik_solver)

# Look for the required libraries
find_library(NLOPT_LIBRARY nlopt)
find_path(NLOPT_INCLUDE_DIR nlopt.hpp)
find_package(trac_ik_lib REQUIRED)
find_package(orocos_kdl REQUIRED)

# # Find TRAC-IK library and include directories
find_path(TRAC_IK_INCLUDE_DIR trac_ik/trac_ik.hpp PATHS /usr/local/include /usr/include)
find_library(TRAC_IK_LIB NAMES trac_ik_lib PATHS /usr/local/lib /usr/lib)

include_directories(${NLOPT_INCLUDE_DIR})
include_directories(${trac_ik_lib_INCLUDE_DIRS})
include_directories(${orocos_kdl_INCLUDE_DIRS})
include_directories(${TORCH_INCLUDE_DIRS})

set(CMAKE_PREFIX_PATH "/home/rvc/colcon_ws/utils/rizon_kinematics/libtorch")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

add_executable(main src/main.cpp 
                    src/rizon_ik_solver.cpp
                    /home/rvc/colcon_ws/utils/rizon_kinematics/src/rizon_kinematics.cpp
                    /home/rvc/colcon_ws/utils/rizon_trac_ik/src/rizon_trac_ik.cpp
                    )

target_link_libraries(main 
    ${NLOPT_LIBRARY} 
    ${TORCH_LIBRARIES} 
    ${orocos_kdl_LIBRARIES} 
    ${TRAC_IK_LIB})

set_property(TARGET main PROPERTY CXX_STANDARD 17)
