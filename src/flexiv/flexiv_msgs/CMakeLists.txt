cmake_minimum_required(VERSION 3.5)
project(flexiv_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/JointPosVel.msg
  msg/Mode.msg
  msg/RobotStates.msg
  msg/ModeController.msg
  msg/RigidBody.msg
  msg/Marker.msg
  msg/MarkerSet.msg
  msg/HanoiGrab.msg
  msg/HanoiPose.msg
  msg/HanoiStates.msg
  msg/ClosestPoint.msg
  msg/DigitalState.msg
  msg/DigitalOutput.msg
  msg/DigitalInput.msg
  msg/FramePose.msg
  msg/RizonFrame.msg
  msg/InteractiveGeometry.msg
  msg/InteractiveGeometryArray.msg
  msg/RawPhone.msg
  srv/AddThreeInts.srv
  srv/SetMode.srv
  srv/SpawnRizonVisual.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
