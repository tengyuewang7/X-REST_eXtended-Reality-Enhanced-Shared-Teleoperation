Flexiv CMake Tools
==================

Library of cmake functions and macros to simplify the build process

* FlexivBuildThirdParty.cmake     - run a bash script to build a library from source
* FlexivGitVersion.cmake          - define cmake variables for git branch and commit hash
* FlexivInstallLibrary.cmake      - create and install a cmake package (include, lib, target, config)
* git_submodule_update_init.cmake - run git submodule update init
* set_default_build_type          - set CMAKE_BUILD_TYPE to release, if not set


How To Build for QNX
--------------------

QNX provides compilers q++/qcc, based on g++/gcc. It support c++14. QNX is unix-like, and conforms to posix.  
See the [QNX c library and utilities reference online documentation](http://www.qnx.com/developers/docs/7.0.0/#com.qnx.doc.qnxsdp.nav/topic/references.html)  


On a linux host machine, use cmake with q++/qcc to compile a project for a QNX neutrino target.  
To do this cross compiling, we need a [cmake toolchain] for QNX.  
Note: This assumes that that qnx momentics has already been installed.

    source ~/qnx700/qnxsdp-env.sh
    mkdir build && cd build
    cmake .. -DCMAKE_TOOLCHAIN_FILE=/path-to-flexiv_cmake_tools/qnx_toolchain.cmake
    make


Using QNX specific c++ code
---------------------------
In source code, use the following for QNX specific code:

    #ifdef __QNX__
        // code that should only be compiled for qnx
    #endif


Using QNX specific cmake
------------------------
In CMakeLists.txt, use the following for QNX specific stuff:

    if(${CMAKE_SYSTEM_NAME} STREQUAL QNX)
        # code that should only be run when compiling for qnx
    endif()


<!-- references -->
[cmake toolchain]:https://cmake.org/cmake/help/latest/manual/cmake-toolchains.7.html