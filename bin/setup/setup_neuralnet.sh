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

# Purpose: NeuralNet setup
# Date:    2014-08-26

##############################################################################
#                         START DO NOT MODIFY                                #
##############################################################################
SCRIPT_PATH="`dirname \"$0\"`"
SCRIPT_PATH="`( cd \"$SCRIPT_PATH\" && pwd )`"
##############################################################################
#                          END DO NOT MODIFY                                 #
##############################################################################

# Add the relative path from this script to the helpers folder.
pushd "${SCRIPT_PATH}/helpers/" > /dev/null

##############################################################################
#                         START DO NOT MODIFY                                #
##############################################################################
if [ ! -f "helper_functions.sh" ]; then
    echo "Could not find helper_functions.sh. Are we in the bash helpers folder?"
    exit 1;
fi

# Import our common files
source "helper_functions.sh"
source "helper_paths.sh"
source "helper_definitions.sh"

# Get out of the bash helpers folder.
popd > /dev/null
##############################################################################
#                          END DO NOT MODIFY                                 #
##############################################################################

#Source this package's configuration
source_conf "general.conf"
source_conf "neuralnet.conf"

# Variables
neuralnet_pkg=`echo $NEURALNET_URL|awk -F/ '{print $NF}'`  # get the package name from the url

# Check to see if neuralnet has been built already
function check_neuralnet_built()
{
    # Check for a library that's created when neuralnet is built   
    fname=$(find "$NEURALNET_BUILD_DIR" -iname libneuralnet* 2>/dev/null)
    if [ -f "$fname" ]; then
        return $TRUE
    fi
    return $FALSE
}

# Download the package to env/downloads
function download_neuralnet()
{

    neuralnet_pkg_path="$DOWNLOADS_DIR/$neuralnet_pkg"

    if [ -f "$neuralnet_pkg_path" ]; then
        echo "- NeuralNet package already exists ('$neuralnet_pkg_path') -- skipping download."
        return
    fi

    echo "Downloading $neuralnet_pkg to $neuralnet_pkg_path"
    download_file "$NEURALNET_URL" "$neuralnet_pkg_path"
}

# Unpack to the build directory specified in install.conf
function unpack_neuralnet()
{
    # Create directory and unpack
    if check_directory_exists "$NEURALNET_BUILD_DIR"; then
        echo "- NeuralNet is already unpacked to '$NEURALNET_BUILD_DIR' -- skipping."
        return
    fi

    echo "Unpacking neuralnet to $NEURALNET_BUILD_DIR (this may take a minute...)"
    # TODO: Do we need to remove the dir if it already exists?
    create_directory_if_noexist $NEURALNET_BUILD_DIR

    # Unzip
    pushd "$NEURALNET_PACKAGE_DIR" > /dev/null
    unzip "$DOWNLOADS_DIR/$neuralnet_pkg" || { echo "- ERROR: Failed to unpack NeuralNet"; exit 1; }
    #mv */* . - todo, investigate cleaner build with this
    popd > /dev/null
}

# Patch the neural network to include Atil and Brian's mods
function patch_neuralnet()
{
    pushd "$NEURALNET_BUILD_DIR" > /dev/null
	
    # Patch them
    patch -p3 < "$SETUP_DIR/patches/neuralNet/nnPatch.diff" || { echo "- ERROR: Failed to patch NeuralNet"; exit 1; }
    
    # Add some additional functions (Brian, 1/7/15)
    patch -p6 < "$SETUP_DIR/patches/neuralNet/NNPatch2_1.patch" || { echo "- ERROR: Failed to patch NeuralNet, 2nd patch"; exit 1; }
    patch -p6 < "$SETUP_DIR/patches/neuralNet/NNPatch2_2.patch" || { echo "- ERROR: Failed to patch NeuralNet, 3rd patch"; exit 1; }
    
    popd > /dev/null
}

# Build the package under the build directory specified in in install.conf
function build_neuralnet()
{

    echo "- Building NeuralNet under $NEURALNET_BUILD_DIR"
    pushd "$NEURALNET_BUILD_DIR" > /dev/null

    # Perform the build
    # If you turn double precision on, turn it on in inc.CMakeJsonCPP.txt as well for the NTRT build
    "$ENV_DIR/bin/cmake" . -G "Unix Makefiles" \
        -DBUILD_SHARED_LIBS=OFF \
        -DBUILD_EXTRAS=ON \
        -DCMAKE_INSTALL_PREFIX="$NEURALNET_INSTALL_PREFIX" \
        -DCMAKE_C_FLAGS="-fPIC" \
        -DCMAKE_C_COMPILER="gcc" \
        -DCMAKE_CXX_COMPILER="g++" \
        -DCMAKE_CXX_FLAGS="-fPIC" \
        -DCMAKE_EXE_LINKER_FLAGS="-fPIC" \
        -DCMAKE_MODULE_LINKER_FLAGS="-fPIC" \
        -DCMAKE_SHARED_LINKER_FLAGS="-fPIC" \
        -DUSE_DOUBLE_PRECISION=OFF \
        -DCMAKE_INSTALL_NAME_DIR="$NEURALNET_INSTALL_PREFIX" || { echo "- ERROR: CMake for NeuralNet failed."; exit 1; }
    #If you turn this on, turn it on in inc.CMakeJsonCPP.txt as well for the NTRT build
    # Additional neuralnet options: 
    # -DFRAMEWORK=ON
    # -DBUILD_DEMOS=ON

    make || { echo "- ERROR: NeuralNet build failed"; exit 1; }

    popd > /dev/null
}

# Install the package under the package install prefix from install.conf
function install_neuralnet()
{

    echo "- Installing NeuralNet under $NEURALNET_INSTALL_PREFIX"

    pushd "$NEURALNET_BUILD_DIR" > /dev/null

    make install || { echo "Install failed -- maybe you need to use sudo when running setup?"; exit 1; }

    popd > /dev/null
}

# Create symlinks under env for building our applications and IDE integration
function env_link_neuralnet()
{

    # Build
    pushd "$ENV_DIR/build" > /dev/null
    rm neuralnet 2>/dev/null   # Note: this will fail if 'neuralnet' is a directory, which is what we want.

    # If we're building under env, use a relative path for the link; otherwise use an absolute one.
    if str_contains "$NEURALNET_BUILD_DIR" "$ENV_DIR"; then
        current_pwd=`pwd`
        rel_path=$(get_relative_path "$current_pwd" "$NEURALNET_BUILD_DIR" )
        ln -s "$rel_path" neuralnet
    else
        ln -s "$NEURALNET_BUILD_DIR" neuralnet  # this links directly to the most recent build...
    fi

    popd > /dev/null

}

function main()
{

    ensure_install_prefix_writable $NEURALNET_INSTALL_PREFIX

    if check_package_installed "$NEURALNET_INSTALL_PREFIX/lib/libneuralnet*"; then
        echo "- NeuralNet is installed under prefix $NEURALNET_INSTALL_PREFIX -- skipping."
        env_link_neuralnet
        return
    fi

    if check_neuralnet_built; then
        echo "- NeuralNet is already built under $NEURALNET_BUILD_DIR -- skipping."
        install_neuralnet
        env_link_neuralnet
        return
    fi

    # @todo: add check neuralnet patched

    if check_file_exists "$NEURALNET_PACKAGE_DIR/CMakeLists.txt"; then
        echo "- NeuralNet is already unpacked to $NEURALNET_BUILD_DIR -- skipping."
        build_neuralnet
        install_neuralnet
        env_link_neuralnet
        return
    fi

    if check_file_exists "$DOWNLOADS_DIR/$neuralnet_pkg"; then
        echo "- NeuralNet package already exists under env/downloads -- skipping download."
        unpack_neuralnet
        patch_neuralnet
        build_neuralnet
        install_neuralnet
        env_link_neuralnet
        return
    fi

    # If we haven't returned by now, we have to do everything
    download_neuralnet
    unpack_neuralnet
    patch_neuralnet
    build_neuralnet
    install_neuralnet
    env_link_neuralnet

}


main
