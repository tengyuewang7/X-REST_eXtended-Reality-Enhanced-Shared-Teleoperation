#!/bin/bash
# Download and install the pre-compiled binary package of Open3D for x86_64 processor, without CUDA.

echo "Start: install_open3d_bin_x64.sh"


# Download, build, and install
cd $DOWNLOAD_DIR
VER=0.15.1
FOLDER="open3d-devel-linux-x86_64-cxx11-abi-$VER"
FILE="$FOLDER.tar.xz"

# Check if folder already exists
if [ -d $FOLDER ]
then
    echo "$FILE already exists"
else
    # Download binary package
    URL="https://github.com/isl-org/Open3D/releases/download/v$VER/$FILE"
    echo "-- Downloading: $URL"
    wget -O $FILE $URL --no-clobber --quiet --show-progress --progress=bar:force 2>&1
    echo "-- Extracting: $FILE"
    tar -xf "$FILE"
fi

# Move binaries to install dir and rename
cp -r $FOLDER/* $INSTALL_DIR/
rm -rf $FOLDER

# Back to original path
cd $SCRIPT_ROOT
echo "Finished: install_open3d_bin_x64.sh"

