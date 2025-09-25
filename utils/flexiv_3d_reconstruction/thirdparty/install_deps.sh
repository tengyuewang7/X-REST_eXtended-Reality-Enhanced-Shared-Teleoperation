#!/bin/bash
# Install dependencies requried by this project

# Exit on failure
set -e

# Absolute path of this script
export SCRIPT_ROOT="$(dirname $(readlink -f $0))"
export DOWNLOAD_DIR="$SCRIPT_ROOT/download"
export INSTALL_DIR="$SCRIPT_ROOT/../install"
echo "Download dependencies to: $DOWNLOAD_DIR"
echo "Install dependencies to: $INSTALL_DIR"
mkdir -p $DOWNLOAD_DIR
mkdir -p $INSTALL_DIR


# Dependencies to install using apt packages
SUDO=${SUDO:=sudo} # SUDO=command in docker (running as root, sudo not available)

dependencies=(
    # Tools
    wget
    # Open3D deps
    xorg-dev
    libglu1-mesa-dev
    python3-dev
    # Filament build-from-source deps
    libsdl2-dev
    libc++-12-dev
    libc++abi-12-dev
    ninja-build
    libxi-dev
    # OpenBLAS build-from-source deps
    gfortran
    # ML deps
    libtbb-dev
    # Headless rendering deps
    libosmesa6-dev
    # RealSense deps
    libudev-dev
    autoconf
    libtool
)

$SUDO apt update
for package in "${dependencies[@]}"; do
    $SUDO apt install -y "$package"
done


# Dependencies to build from source and install to repo_root/install
# Open3d 0.15.1 binary package for x64
bash install_open3d_bin_x64.sh;

echo "Build and install dependencies complete"
