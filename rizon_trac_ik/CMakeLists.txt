# cmake_minimum_required(VERSION 3.10)
# project(MyProject)

# # Find Eigen3
# find_package(Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIR})

# # Find orocos_kdl
# find_package(orocos_kdl REQUIRED)
# include_directories(${orocos_kdl_INCLUDE_DIRS})

# # Find TRAC-IK library and include directories
# find_path(TRAC_IK_INCLUDE_DIR trac_ik/trac_ik.hpp PATHS /usr/local/include /usr/include)
# find_library(TRAC_IK_LIB NAMES trac_ik_lib PATHS /usr/local/lib /usr/lib)

# # Include the necessary directories
# include_directories(
#     ${TRAC_IK_INCLUDE_DIR}
#     ${orocos_kdl_INCLUDE_DIRS}
# )

# # Add executable
# add_executable(my_executable src/main.cpp src/rizon_trac_ik.cpp)

# # Link the libraries
# target_link_libraries(my_executable
#     ${TRAC_IK_LIB}
#     ${orocos_kdl_LIBRARIES}
#     Eigen3::Eigen
# )


cmake_minimum_required(VERSION 3.10)
project(trac_ik_solver)

# Find orocos_kdl
find_package(orocos_kdl REQUIRED)

# # Find TRAC-IK library and include directories
find_path(TRAC_IK_INCLUDE_DIR trac_ik/trac_ik.hpp PATHS /usr/local/include /usr/include)
find_library(TRAC_IK_LIB NAMES trac_ik_lib PATHS /usr/local/lib /usr/lib)

# Add executable
add_executable(main src/main.cpp src/rizon_trac_ik.cpp)

# Link the libraries
target_link_libraries(main
    ${TRAC_IK_LIB}
    ${orocos_kdl_LIBRARIES}
)
