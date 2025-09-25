cmake_minimum_required(VERSION 3.5)
project(to_unity_cpp)

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

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  flexiv_msgs
  trac_ik_lib
  orocos_kdl
  trajectory_msgs
  ur_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

add_executable(rizon_ik src/rizon_ik.cpp)
add_executable(rizon_ik_forward src/rizon_ik_forward.cpp)
add_executable(ur3e_control src/ur3e_control.cpp)
add_executable(rizon_model src/rizon_model.cpp)



include_directories(include)

# target_link_libraries(rizon_test
#   ${NLOPT_LIBRARIES}
# )

ament_target_dependencies(rizon_ik
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(rizon_ik_forward
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(ur3e_control
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(rizon_model
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS 
  rizon_ik
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  rizon_ik_forward
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  ur3e_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  rizon_model
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
