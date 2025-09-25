cmake_minimum_required(VERSION 3.10)
project(scripts)

# 设置 Open3D 的安装路径
set(Open3D_DIR /home/rvc/colcon_ws/utils/flexiv_3d_reconstruction/install/lib/cmake/Open3D)
set(Open3D_INCLUDE_DIR /home/rvc/colcon_ws/utils/flexiv_3d_reconstruction/install/include)

# 设置 flexiv_3d_reconstruction 库的路径
set(FLEXIV_3D_RECONSTRUCTION_LIB_DIR /home/rvc/colcon_ws/utils/flexiv_3d_reconstruction/build)
set(FLEXIV_3D_RECONSTRUCTION_INCLUDE_DIR /home/rvc/colcon_ws/utils/flexiv_3d_reconstruction/include)

# 查找 Open3D 库
find_package(Open3D REQUIRED)
find_package(Threads REQUIRED)
# 添加包含路径
include_directories(${Open3D_INCLUDE_DIR})
include_directories(${FLEXIV_3D_RECONSTRUCTION_INCLUDE_DIR})

# 查找并链接 flexiv_3d_reconstruction 库
find_library(FLEXIV_3D_RECONSTRUCTION_LIB NAMES flexiv_3d_reconstruction HINTS ${FLEXIV_3D_RECONSTRUCTION_LIB_DIR})

# # 创建可执行文件并链接库
# add_executable(scripts main.cpp)
# target_link_libraries(scripts ${FLEXIV_3D_RECONSTRUCTION_LIB} Open3D::Open3D Threads::Threads) 

# 创建可执行文件并链接库
add_executable(aaa aaa.cpp)
target_link_libraries(aaa ${FLEXIV_3D_RECONSTRUCTION_LIB} Open3D::Open3D Threads::Threads) 

