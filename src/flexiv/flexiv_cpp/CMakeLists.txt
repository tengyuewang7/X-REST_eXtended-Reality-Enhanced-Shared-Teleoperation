cmake_minimum_required(VERSION 3.5)
project(flexiv_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread ${TORCH_CXX_FLAGS}")

list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/libtorch)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(flexiv_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Torch REQUIRED) 
find_package(trac_ik_lib REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

include_directories(/home/rvc/colcon_ws/utils/assimp/include)

# 添加 Assimp 的库文件路径
link_directories(/home/rvc/colcon_ws/utils/assimp/bin)

#################################################################
# executable flexiv_robot
add_executable(flexiv_robot src/flexiv_robot.cpp)
ament_target_dependencies(
	flexiv_robot
    rclcpp
    flexiv_msgs
)

target_include_directories(flexiv_robot
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/fladd_executablexiv_rdk/include
        ${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk/thirdparty/eigen3
)

target_link_libraries(flexiv_robot 
		${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk/lib/libFlexivRdk.x86_64-linux-gnu.a
		pthread
		)

###################################################################
# executable robot_test
add_executable(robot_test src/robot_test.cpp)
ament_target_dependencies(
	robot_test
    rclcpp
    flexiv_msgs
)

target_include_directories(robot_test 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/fladd_executablexiv_rdk/include
        ${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk/thirdparty/eigen3
)

target_link_libraries(robot_test 
		${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk/lib/libFlexivRdk.x86_64-linux-gnu.a
		pthread
		)

###################################################################

# executable flexiv_unity

find_library(NLOPT_LIBRARY nlopt)
find_path(NLOPT_INCLUDE_DIR nlopt.hpp)
# 包含NLopt的头文件
include_directories(${NLOPT_INCLUDE_DIR}) 
set(CMAKE_PREFIX_PATH "/home/rvc/colcon_ws/utils/rizon_kinematics/libtorch")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

add_executable(flexiv_unity src/flexiv_unity.cpp 
               /home/rvc/colcon_ws/utils/rizon_kinematics/src/rizon_kinematics.cpp 
               /home/rvc/colcon_ws/utils/rizon_ik_solver/src/rizon_ik_solver.cpp 
               /home/rvc/colcon_ws/utils/rizon_trac_ik/src/rizon_trac_ik.cpp)

# 链接NLopt库
target_link_libraries(flexiv_unity ${NLOPT_LIBRARY})
# target_link_libraries(flexiv_unity "${TORCH_LIBRARIES}")

ament_target_dependencies(
	flexiv_unity
    rclcpp
    flexiv_msgs
    orocos_kdl
    trac_ik_lib
)

target_include_directories(flexiv_unity
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/fladd_executablexiv_rdk/include
        ${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk/thirdparty/eigen3
        ${TORCH_INCLUDE_DIRS}
)

target_link_libraries(flexiv_unity 
		${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk/lib/libFlexivRdk.x86_64-linux-gnu.a
		pthread
        assimp
        ${TORCH_LIBRARIES}
		)
set_property(TARGET flexiv_unity PROPERTY CXX_STANDARD 17)


# #################################################################

# executable unity_decision_making
add_executable(unity_decision_making src/unity_decision_making.cpp)
ament_target_dependencies(
	unity_decision_making
    rclcpp
    flexiv_msgs
)

target_include_directories(unity_decision_making
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/fladd_executablexiv_rdk/include
        ${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk/thirdparty/eigen3
)

target_link_libraries(unity_decision_making 
		${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk/lib/libFlexivRdk.x86_64-linux-gnu.a
		pthread
		)

# #################################################################

# # executable flexiv_unity_nrt

# find_library(NLOPT_LIBRARY nlopt)
# find_path(NLOPT_INCLUDE_DIR nlopt.hpp)
# # 包含NLopt的头文件
# include_directories(${NLOPT_INCLUDE_DIR}) 
# set(CMAKE_PREFIX_PATH "/home/rvc/colcon_ws/utils/rizon_kinematics/libtorch")
# find_package(Torch REQUIRED)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

# add_executable(flexiv_unity_nrt src/flexiv_unity_nrt.cpp 
#                /home/rvc/colcon_ws/utils/rizon_kinematics/src/flexiv_kinematics.cpp 
#                /home/rvc/colcon_ws/utils/rizon_ik_solver/src/rizon_nlopt_ik.cpp 
#                /home/rvc/colcon_ws/utils/rizon_trac_ik/src/rizon_trac_ik.cpp)
# # 链接NLopt库
# target_link_libraries(flexiv_unity_nrt ${NLOPT_LIBRARY})
# # target_link_libraries(flexiv_unity_nrt "${TORCH_LIBRARIES}")

# ament_target_dependencies(
# 	flexiv_unity_nrt
#     rclcpp
#     flexiv_msgs
#     orocos_kdl
#     trac_ik_lib
# )

# target_include_directories(flexiv_unity_nrt
#     PUBLIC 
#         ${CMAKE_CURRENT_SOURCE_DIR}/include
#         ${CMAKE_CURRENT_SOURCE_DIR}/fladd_executablexiv_rdk/include
#         ${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk/thirdparty/eigen3
#         ${TORCH_INCLUDE_DIRS}
# )

# target_link_libraries(flexiv_unity_nrt 
# 		${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk/lib/libFlexivRdk.x86_64-linux-gnu.a
# 		pthread
#         ${TORCH_LIBRARIES}
# 		)
# set_property(TARGET flexiv_unity_nrt PROPERTY CXX_STANDARD 17)

#################################################################

# executable flexiv_test

find_library(NLOPT_LIBRARY nlopt)
find_path(NLOPT_INCLUDE_DIR nlopt.hpp)
# 包含NLopt的头文件
include_directories(${NLOPT_INCLUDE_DIR}) 
set(CMAKE_PREFIX_PATH "/home/rvc/colcon_ws/utils/rizon_kinematics/libtorch")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

add_executable(flexiv_test src/flexiv_test.cpp 
               /home/rvc/colcon_ws/utils/rizon_kinematics/src/rizon_kinematics.cpp 
               /home/rvc/colcon_ws/utils/rizon_ik_solver/src/rizon_ik_solver.cpp 
               /home/rvc/colcon_ws/utils/rizon_trac_ik/src/rizon_trac_ik.cpp)

# 链接NLopt库
target_link_libraries(flexiv_test ${NLOPT_LIBRARY})
# target_link_libraries(flexiv_test "${TORCH_LIBRARIES}")

ament_target_dependencies(
	flexiv_test
    rclcpp
    flexiv_msgs
    orocos_kdl
    trac_ik_lib
)

target_include_directories(flexiv_test
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/fladd_executablexiv_rdk/include
        ${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk/thirdparty/eigen3
        ${TORCH_INCLUDE_DIRS}
)

target_link_libraries(flexiv_test 
		${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk/lib/libFlexivRdk.x86_64-linux-gnu.a
		pthread
        ${TORCH_LIBRARIES}
        assimp
		)
set_property(TARGET flexiv_test PROPERTY CXX_STANDARD 17)

#################################################################

# executable flexiv_reconstruction
# 设置 Open3D 的安装路径
set(Open3D_DIR /home/rvc/colcon_ws/utils/flexiv_3d_reconstruction/install/lib/cmake/Open3D)
set(Open3D_INCLUDE_DIR /home/rvc/colcon_ws/utils/flexiv_3d_reconstruction/install/include)

# 设置 flexiv_3d_reconstruction 库的路径
set(FLEXIV_3D_RECONSTRUCTION_LIB_DIR /home/rvc/colcon_ws/utils/flexiv_3d_reconstruction/build)
set(FLEXIV_3D_RECONSTRUCTION_INCLUDE_DIR /home/rvc/colcon_ws/utils/flexiv_3d_reconstruction/include)

# 查找 Open3D 库
find_package(Open3D REQUIRED)

# 查找 pthread 库
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)


# 添加包含路径
include_directories(${Open3D_INCLUDE_DIR})
include_directories(${FLEXIV_3D_RECONSTRUCTION_INCLUDE_DIR})

# 查找并链接 flexiv_3d_reconstruction 库
find_library(FLEXIV_3D_RECONSTRUCTION_LIB NAMES flexiv_3d_reconstruction HINTS ${FLEXIV_3D_RECONSTRUCTION_LIB_DIR})


add_executable(flexiv_reconstruction src/flexiv_reconstruction.cpp /home/rvc/colcon_ws/utils/flexiv_3d_reconstruction/scripts/environment_reconstruction.cpp)
ament_target_dependencies(
	flexiv_reconstruction
    rclcpp
    flexiv_msgs
    sensor_msgs
    OpenCV
    cv_bridge
    message_filters
)
target_link_libraries(flexiv_reconstruction ${FLEXIV_3D_RECONSTRUCTION_LIB} Open3D::Open3D Threads::Threads)


# add_subdirectory(/home/rvc/colcon_ws/utils/flexiv_3d_reconstruction)

message(${CMAKE_BINARY_DIR})

# add_subdirectory(/home/rvc/colcon_ws/utils/flexiv_3d_reconstruction ${CMAKE_BINARY_DIR}/flexiv_reconstruction)


#################################################################

# executable flexiv_trac_ik
add_executable(flexiv_trac_ik src/flexiv_trac_ik.cpp 
               /home/rvc/colcon_ws/utils/rizon_trac_ik/src/rizon_trac_ik.cpp)


ament_target_dependencies(
	flexiv_trac_ik
    rclcpp
    flexiv_msgs
    orocos_kdl
    trac_ik_lib
)

target_include_directories(flexiv_trac_ik
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/fladd_executablexiv_rdk/include
        ${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk/thirdparty/eigen3
        ${TORCH_INCLUDE_DIRS}
)

target_link_libraries(flexiv_trac_ik 
		${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk/lib/libFlexivRdk.x86_64-linux-gnu.a
		pthread
        assimp
        ${TORCH_LIBRARIES}
		)

#################################################################
# executable keyboard_control
add_executable(keyboard_control src/keyboard_control.cpp)
ament_target_dependencies(
	keyboard_control
    rclcpp
    flexiv_msgs
)

target_include_directories(keyboard_control
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/fladd_executablexiv_rdk/include
        ${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk/thirdparty/eigen3
)

target_link_libraries(keyboard_control 
		${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk/lib/libFlexivRdk.x86_64-linux-gnu.a
		pthread
		)


#################################################################

# executable test_ros2
add_executable(test_ros2 src/test_ros2.cpp)
ament_target_dependencies(
	test_ros2
    rclcpp
    flexiv_msgs
)

target_include_directories(test_ros2
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/fladd_executablexiv_rdk/include
        ${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk/thirdparty/eigen3
)

target_link_libraries(test_ros2 
		${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk/lib/libFlexivRdk.x86_64-linux-gnu.a
		pthread
		)

#################################################################
# executable flexiv_mesh

set(CMAKE_PREFIX_PATH "/home/rvc/colcon_ws/utils/rizon_kinematics/libtorch")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

add_executable(flexiv_mesh src/flexiv_mesh.cpp
                            /home/rvc/colcon_ws/utils/rizon_kinematics/src/rizon_kinematics.cpp)
ament_target_dependencies(
	flexiv_mesh
    rclcpp
    flexiv_msgs
    sensor_msgs
    geometry_msgs
)

# include_directories(/home/rvc/colcon_ws/utils/assimp/include)

# # 添加 Assimp 的库文件路径
# link_directories(/home/rvc/colcon_ws/utils/assimp/bin)

target_include_directories(flexiv_mesh
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/fladd_executablexiv_rdk/include
        ${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk/thirdparty/eigen3
)

target_link_libraries(flexiv_mesh 
		${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk/lib/libFlexivRdk.x86_64-linux-gnu.a
		pthread
        assimp
        ${TORCH_LIBRARIES}
		)

set_property(TARGET flexiv_unity PROPERTY CXX_STANDARD 17)

###################################################################

# ==================================================================================
# Add executable file name; ex: states_test
install(TARGETS
  flexiv_robot
  robot_test
  flexiv_unity
  unity_decision_making
#   flexiv_unity_nrt
  flexiv_test
  flexiv_reconstruction
  flexiv_trac_ik
  keyboard_control
  test_ros2
  flexiv_mesh
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
