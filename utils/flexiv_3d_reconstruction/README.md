# Flexiv 3D Reconstruction

3D reconstruction library written in C++ based on Open3D. A scene is reconstructed using depth cameras, point cloud describing the scene can be extracted for secondary development.

## Coding style

This project uses the [Google C++ Style](https://google.github.io/styleguide/cppguide.html#Naming) to be consistent with Open3D.

## Link to this project from another project

Assume the parent project is called ``example_parent_project``.

1. Add ``flexiv_3d_reconstruction`` as a submodule of the parent project:

        cd example_parent_project/thirdparty
        git submodule add https://bitbucket.org/FlexivTeam/flexiv_3d_reconstruction.git

2. Initialize all submodules of both parent and child projects:

        cd example_parent_project
        git submodule update --init --recursive

3. Install dependencies of ``flexiv_3d_reconstruction``:

        cd example_parent_project/thirdparty/flexiv_3d_reconstruction/thirdparty
        bash install_deps.sh

4. Use ``cmake-gui`` to configure the top-level CMake:

        cd example_parent_project
        mkdir build && cd build
        cmake-gui .. &

5. Configure, then check that ``Open3D_ROOT`` is set correctly, which should be automatically set to path:

        example_parent_project/thirdparty/flexiv_3d_reconstruction/install

6. Configure again, then generate. You may encounter errors, please refer to [flexiv_teleoperation](https://bitbucket.org/FlexivTeam/flexiv_teleoperation) on how to properly configure top-level and app-level CMakeLists.

## Build the project standalone

1. Update all submodules

        git submodule update --init --recursive

2. Build and install dependencies to ``flexiv_3d_reconstruction/install``:

        cd flexiv_3d_reconstruction/thirdparty
        bash install_deps.sh

3. Use ``cmake-gui`` to configure the top-level CMake:

        cd flexiv_3d_reconstruction
        mkdir build && cd build
        cmake-gui .. &

4. Configure, then check that ``Open3D_ROOT`` is set to the correct path.
5. Configure again, then generate.
6. Build base library and all examples:

        make -j4

   Installing the base library to ``CMAKE_INSTALL_PREFIX`` is **optional**:

        make install

## Run the example

Instructions are provided in [examples README](examples/README.md)
