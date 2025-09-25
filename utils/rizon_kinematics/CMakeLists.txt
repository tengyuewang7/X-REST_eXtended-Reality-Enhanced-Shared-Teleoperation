cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(rizon_kinematics)

set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR})
# list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_PREFIX_PATH "/home/rvc/colcon_ws/utils/rizon_kinematics/libtorch")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

include_directories(/home/rvc/colcon_ws/utils/assimp/include)
link_directories(/home/rvc/colcon_ws/utils/assimp/bin)

add_executable(main src/main.cpp src/rizon_kinematics.cpp) 
target_link_libraries(main 
    "${TORCH_LIBRARIES}", 
    assimp)

set_property(TARGET main PROPERTY CXX_STANDARD 17)
