cmake_minimum_required (VERSION 3.4)

project (xbox)

find_package(Threads REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/eigen3/Eigen)

add_executable(main src/main.cpp)
add_executable(test src/test.cpp)

target_link_libraries(main pthread)
target_link_libraries(test pthread)
install(TARGETS
  main
  test
  DESTINATION lib/${PROJECT_NAME})
