cmake_minimum_required(VERSION 3.10)

# ===================================================================
#      PROJECT SETUP
# ===================================================================
# set the project name
project(flexiv_3d_reconstruction VERSION 0.1.0)

set(BUILD_SHARED_LIBS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/tools/set_default_build_type.cmake)
set_default_build_type()

# Options
option(BUILD_EXAMPLES "Build all examples" OFF)
option(BUILD_CUDA_MODULE "Build with CUDA modules for GPU acceleration" OFF)

# ===================================================================
#      PROJECT DEPENDENCIES
# ===================================================================
# Open3D
set(Open3D_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/install/" CACHE PATH "Open3D_ROOT path")
if (NOT EXISTS "${Open3D_ROOT}")
  message(FATAL_ERROR "Open3D_ROOT=${Open3D_ROOT} does not exist.")
endif()

find_package(Open3D 0.15.1 REQUIRED HINTS ${Open3D_ROOT} NO_DEFAULT_PATH)
if (${Open3D_FOUND})
    message(STATUS "Found Open3D: ${Open3D_DIR}")
endif()

# CUDA for GPU acceleration
if (${BUILD_CUDA_MODULE})
  find_package(CUDAToolkit REQUIRED)
endif()

# pthread
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# ===================================================================
#      PROJECT CONFIG
# ===================================================================
# create a header file that defines the library version and git version


# ===================================================================
#      PROJECT LIBRARIES
# ===================================================================
# Create base library from source
file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(${PROJECT_NAME} 
    ${SRC_FILES}
)

# Create an alias of the library using flexiv namespace, 
# to imitate the install target which uses flexiv namespace.
add_library(flexiv::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Set location of include headers
# public includes are used to build and are installed
# private includes are used only to build, and will not be installed
target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include> # the headers will be installed in the include folder
)

target_link_libraries(${PROJECT_NAME} 
    PUBLIC
        Open3D::Open3D
        Threads::Threads
)

# Link CUDA libraries if enabled
if (${BUILD_CUDA_MODULE})
    target_link_libraries(${PROJECT_NAME} 
    PUBLIC
        CUDA::cudart
        CUDA::cublas
        CUDA::cufft
        CUDA::cusolver
        CUDA::curand
        CUDA::nppicc
        CUDA::nppial
        CUDA::nppist
        CUDA::nppidei
        CUDA::nppig
        CUDA::nppitc
        CUDA::npps
    )
endif()

# Set cmake compile features and options, this replaces the CMAKE_CXX_FLAGS
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
target_compile_options(${PROJECT_NAME} PRIVATE -Werror)


# ===================================================================
#      PROJECT APPS
# ===================================================================
# Add all app-level CMake projects as subdirectories, 
# no need to configure them individually
# if (${BUILD_EXAMPLES})
#     add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/examples)
# endif()

# ===================================================================
#      UNIT TESTS
# ===================================================================
# Use add_test(<executable target>) to run an executable with "make test"
# CTest automatically creates a BUILD_TESTING option (default ON).
# Note: This should be called from the top level CMakeLists.txt
#include( CTest )

# TODO =====
# if (${BUILD_TESTING})
#     # create executable from source
#     set(TEST_FILES ${CMAKE_CURRENT_SOURCE_DIR}/test/friction/friction_test.cc)
#     add_executable(${PROJECT_NAME}_test ${TEST_FILES})

#     # find all packages
#     # we cannot use find_package(${PROJECT_NAME}) because it hasn't been created yet
#     find_package(GTest REQUIRED)

#     # link the libraries
#     target_link_libraries(${PROJECT_NAME}_test
#             flexiv::${PROJECT_NAME}
#             GTest::GTest
#             GTest::Main
#             -pthread
#     )

#     # make test will run the ${PROJECT_NAME}_test executable
#     add_test(NAME ${PROJECT_NAME}_test COMMAND ${PROJECT_NAME}_test
#              WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test)
# endif()
# END TODO =======
