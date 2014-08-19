#!/bin/bash

# Copyright © 2012, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# All rights reserved.
# 
# The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
# under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0.
# 
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
# either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

# Purpose: jsoncpp setup
# Date:    2014-08-18

# Source our common setup code
local_setup_path="`dirname \"$0\"`"                # relative
base_dir="`( cd \"$local_setup_path/../../\" && pwd )`"  # absolutized and normalized
source "$base_dir/bin/setup/setup_common.sh"

#Source this package's configuration
source_conf("jsoncpp.conf")

SCRIPT_PATH="`dirname \"$0\"`"                  # relative
SCRIPT_PATH="`( cd \"$SCRIPT_PATH\" && pwd )`"  # absolutized and normalized
setup_dir="$SCRIPT_PATH"

# Variables
jsoncpp_pkg=`echo $JSONCPP_URL|awk -F/ '{print $NF}'`  # get the package name from the url

# Constants
TRUE=0  # Yes, TRUE is 0 (e.g., no errors)
FALSE=1 # FALSE is non-zero

function ensure_install_prefix_writable() {
    touch "$JSONCPP_INSTALL_PREFIX/tensegrity.deleteme" 2>/dev/null \
        || { echo "Install prefix '$JSONCPP_INSTALL_PREFIX' is not writable -- please use sudo or execute as root."; exit 1; }
    rm "$JSONCPP_INSTALL_PREFIX/tensegrity.deleteme"
}

# Check if the package is already installed in the location specified in install.conf 
function check_jsoncpp_installed() {
    count_libs=$(count_files "$JSONCPP_INSTALL_PREFIX/lib/libjsoncpp*")
    if [ "$count_libs" == "0" ]; then
        return $FALSE
    fi
    return $TRUE
}

# Check to see if jsoncpp has been built already
function check_jsoncpp_built() {
    # Check for a library that's created when jsoncpp is built   
    fname=$(find "$JSONCPP_BUILD_DIR" -iname libjsoncpp.* 2>/dev/null)
    if [ -f "$fname" ]; then
        return $TRUE
    fi
    return $FALSE
}

# Check to see if jsoncpp has been unpacked
function check_jsoncpp_unpacked() {
    # The CMakeLists.txt will only exist if it's been unpacked.
    if [ -f "$JSONCPP_PACKAGE_DIR/CMakeLists.txt" ]; then
        return $TRUE
    fi
    return $FALSE
}

# Determine if the package exists under env/downloads
function check_jsoncpp_downloaded() {
    if [ -f "$downloads_dir/$jsoncpp_pkg" ]; then
        return $TRUE
    fi
    return $FALSE
}

# Download the package to env/downloads
function download_jsoncpp() {

    jsoncpp_pkg_path="$downloads_dir/$jsoncpp_pkg"

    if [ -f "$jsoncpp_pkg_path" ]; then
        echo "- jsoncpp package already exists ('$jsoncpp_pkg_path') -- skipping download."
        return
    fi

    echo "Downloading $jsoncpp_pkg to $bullet_pkg_path"
    curl -k -L "$JSONCPP_URL" > "$jsoncpp_pkg_path" || { echo "- ERROR: jsoncpp download failed."; exit 1; }
}

# Unpack to the build directory specified in install.conf
function unpack_jsoncpp() {
    # Create directory and unpack
    if [ -d "$JSONCPP_BUILD_DIR" ]; then
        echo "- jsoncpp is already unpacked to '$JSONCPP_BUILD_DIR' -- skipping."
        return
    fi

    echo "Unpacking jsoncpp to $JSONCPP_BUILD_DIR (this may take a minute...)"
    if [ ! -d "$JSONCPP_BUILD_DIR" ]; then
        # TODO: Do we need to remove the dir if it already exists?
        mkdir -p "$JSONCPP_BUILD_DIR"
    fi

    # Unzip
    pushd "$JSONCPP_BUILD_DIR" > /dev/null
    tar xf "$downloads_dir/$jsoncpp_pkg" --strip 1 || { echo "- ERROR: Failed to unpack jsoncpp"; exit 1; }
    popd > /dev/null
}

# Build the package under the build directory specified in in install.conf
function build_jsoncpp() {
    
    echo "- Building jsoncpp under $JSONCPP_BUILD_DIR"
    pushd "$JSONCPP_BUILD_DIR" > /dev/null
    
    # Drew Sabelhaus Edit 4-28-14
    # Call cmake from a different place depending on if you've custom installed it,
    # or want to use the version that comes with your distribution of linux.
    if [ $USE_DISTRO_CMAKE == 1 ]; then
    CMAKECOMMAND="cmake"
    else
    CMAKECOMMAND="$env_dir/bin/cmake"
    fi

	# Perform the build
	# If you turn double precision on, turn it on in inc.CMakeBullet.txt as well for the NTRT build
    "$env_dir/bin/cmake" . -G "Unix Makefiles" \
    $CMAKECOMMAND . -G "Unix Makefiles" \
        -DBUILD_SHARED_LIBS=OFF \
        -DBUILD_EXTRAS=ON \
        -DCMAKE_INSTALL_PREFIX="$JSONCPP_INSTALL_PREFIX" \
    -DCMAKE_C_FLAGS="-fPIC" \
    -DCMAKE_CXX_FLAGS="-fPIC" \
    -DCMAKE_EXE_LINKER_FLAGS="-fPIC" \
    -DCMAKE_MODULE_LINKER_FLAGS="-fPIC" \
    -DCMAKE_SHARED_LINKER_FLAGS="-fPIC" \
    -DUSE_DOUBLE_PRECISION=OFF \
        -DCMAKE_INSTALL_NAME_DIR="$JSONCPP_INSTALL_PREFIX" || { echo "- ERROR: CMake for jsoncpp failed."; exit 1; }
           
    make || { echo "- ERROR: jsoncpp build failed"; exit 1; }
    
    popd > /dev/null
}

# Install the package under the package install prefix from install.conf
function install_jsoncpp() {
    
    echo "- Installing jsoncpp under $JSONCPP_INSTALL_PREFIX"
    
    pushd "$JSONCPP_BUILD_DIR" > /dev/null

    make install || { echo "Install failed -- maybe you need to use sudo when running setup?"; exit 1; }
    
    popd > /dev/null
}

function main() {
        
    ensure_install_prefix_writable

    if check_jsoncpp_installed; then
	    echo "- jsoncpp is already installed under $JSONCPP_BUILD_DIR -- skipping."
	    return
    fi
   
    if check_jsoncpp_built; then
        echo "- jsoncpp is already built under $JSONCPP_BUILD_DIR -- skipping."
        install_jsoncpp
        return
    fi
    
    if check_jsoncpp_unpacked; then
        echo "- jsoncpp is already unpacked to $JSONCPP_BUILD_DIR -- skipping."
        build_jsoncpp
        install_jsoncpp
        return
    fi
    
    if check_jsoncpp_downloaded; then
        echo "- jsoncpp package already exists under env/downloads -- skipping download."
        unpack_jsoncpp
        build_jsoncpp
        install_jsoncpp
        return
    fi
    
    # If we haven't returned by now, we have to do everything
    download_jsoncpp
    unpack_jsoncpp
    build_jsoncpp
    install_jsoncpp

}


main
