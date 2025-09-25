# This function will call a bash script to build the thirdparty libraries
# It will pass the following  cmake arguments to the bash script:
# - CMAKE_BUILD_TYPE
# - BUILD_SHARED_LIBS
# - CMAKE_PREFIX_PATH
# - CMAKE_INSTALL_PREFIX
# - CMAKE_TOOLCHAIN_FILE
#
# The bash script will be run in ${CMAKE_CURRENT_BINARY_DIR}/thirdparty
#
# Example:
#
#     FlexivBuildThirdParty(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/build_thirdparty.sh)

function(FlexivBuildThirdParty SHELL_SCRIPT)

    message(STATUS "Running: ${SHELL_SCRIPT}")

    FlexivCmakeArgs()

    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/thirdparty)
    execute_process(COMMAND bash ${SHELL_SCRIPT} ${CMAKE_ARGS}
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/thirdparty"
        )

endfunction()

macro(FlexivCmakeArgs)

    unset(CMAKE_ARGS)

    if(DEFINED CMAKE_BUILD_TYPE)
        list(APPEND CMAKE_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
    endif()

    if(DEFINED BUILD_SHARED_LIBS)
        list(APPEND CMAKE_ARGS "-DBUILD_SHARED_LIBS=${BUILD_SHARED_LIBS}")
    endif()

    if(DEFINED CMAKE_PREFIX_PATH)
        list(APPEND CMAKE_ARGS "-DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}")
    endif()

    if(DEFINED CMAKE_INSTALL_PREFIX)
        list(APPEND CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}")
    endif()

    if(DEFINED CMAKE_TOOLCHAIN_FILE)
        list(APPEND CMAKE_ARGS "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
    endif()

    # Don't build test for QNX
    if ( (DEFINED ENV{QNX_TARGET}) AND (DEFINED CMAKE_TOOLCHAIN_FILE) )
        list(APPEND CMAKE_ARGS "-DBUILD_TESTING=OFF")
    endif()

endmacro()
