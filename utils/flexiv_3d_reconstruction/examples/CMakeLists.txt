# This app-level CMake project is meant to be included as a subdirectory
# by the top-level  CMake, do NOT configure this project on its own.

# use cmake 3 which supports targets
cmake_minimum_required(VERSION 3.1.3)

set(EXAMPLE_LIST
    reconstruct_and_extract_point_cloud 
)

foreach(example ${EXAMPLE_LIST})
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example}
        flexiv::flexiv_3d_reconstruction
    )
endforeach()


add_executable(main main.cpp environment_reconstruction.cpp)
target_link_libraries(main
    flexiv::flexiv_3d_reconstruction
)