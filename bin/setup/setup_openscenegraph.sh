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

# Purpose: OpenSceneGraph setup
# Date:    2013-05-01

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
source_conf "openscenegraph.conf"

# Variables
openscenegraph_pkg=`echo $OSG_URL|awk -F/ '{print $NF}'`  # get the package name from the url

# Check to see if openscenegraph has been built already
function check_openscenegraph_built()
{

    # Check for a library that's created when openscenegraph is built   
    fname=$(find "$OSG_BUILD_DIR" -iname libosg.* 2>/dev/null)
    if [ -f "$fname" ]; then
        return $TRUE
    fi
    return $FALSE
}


# Download the package to env/downloads
function download_openscenegraph()
{

    openscenegraph_pkg_path="$DOWNLOADS_DIR/$openscenegraph_pkg"

    if [ -f "$openscenegraph_pkg_path" ]; then
        echo "- OpenSceneGraph package already exists ('$openscenegraph_pkg_path') -- skipping download."
        return
    fi

    echo "Downloading $openscenegraph_pkg to $openscenegraph_pkg_path"
    download_file "$OSG_URL" "$openscenegraph_pkg_path"
}

# Unpack to the build directory specified in install.conf
function unpack_openscenegraph()
{
    # Create directory and unpack
    if check_directory_exists "$OSG_BUILD_DIR"; then
        echo "- OpenSceneGraph is already unpacked to '$OSG_BUILD_DIR' -- skipping."
        return
    fi

    echo "Unpacking openscenegraph to $OSG_BUILD_DIR (this may take a minute...)"
    # TODO: Do we need to remove the dir if it already exists?
    create_directory_if_noexist $OSG_BUILD_DIR

    # Unzip
    pushd "$OSG_BUILD_DIR" > /dev/null
    tar xf "$DOWNLOADS_DIR/$openscenegraph_pkg" --strip 1 || { echo "- ERROR: Failed to unpack OpenSceneGraph."; exit 1; }
    popd > /dev/null
}


# Build the package under the build directory specified in in install.conf
function build_openscenegraph()
{

    echo "- Building OpenSceneGraph under $OSG_BUILD_DIR"
    pushd "$OSG_BUILD_DIR" > /dev/null

    # Perform the build
    "$ENV_DIR/bin/cmake" . -G "Unix Makefiles" \
        -DBUILD_SHARED_LIBS=OFF \
        -DCMAKE_INSTALL_PREFIX="$OSG_INSTALL_PREFIX" \
        -DCMAKE_C_FLAGS="-fPIC" \
        -DCMAKE_CXX_FLAGS="-fPIC" \
        -DCMAKE_EXE_LINKER_FLAGS="-fPIC" \
        -DCMAKE_MODULE_LINKER_FLAGS="-fPIC" \
        -DCMAKE_SHARED_LINKER_FLAGS="-fPIC" \
        -DCMAKE_INSTALL_NAME_DIR="$OSG_INSTALL_PREFIX" || { echo "- ERROR: CMake for OpenSceneGraph failed."; exit 1; }

    # @todo: this doesn't work on OSX...
    #-DCMAKE_C_COMPILER="gcc" \
    #-DCMAKE_CXX_COMPILER="g++" \


    make || { echo "- ERROR: OpenSceneGraph build failed. Attempting to explicitly make from directory."; make_openscenegraph_local; }

    popd > /dev/null
}

function make_openscenegraph_local()
{
    pushd "$OSG_BUILD_DIR" > /dev/null

    make || { echo "Explicit make of OpenSceneGraph failed as well."; exit 1; }

    popd > /dev/null
}

# Install the package under the package install prefix from install.conf
function install_openscenegraph()
{

    echo "- Installing OpenSceneGraph under $OSG_INSTALL_PREFIX"

    pushd "$OSG_BUILD_DIR" > /dev/null

    make install || { echo "Install failed -- maybe you need to use sudo when running setup?"; exit 1; }

    popd > /dev/null
}

# Create symlinks under env for building our applications and IDE integration
function env_link_openscenegraph()
{
    # Note: this doesn't require an env link
    return

    #@todo: ensure that this will work properly and remove the following code if so

    # Build
    pushd "$ENV_DIR/build" > /dev/null
    rm openscenegraph 2>/dev/null   # Note: this will fail if 'openscenegraph' is a directory, which is what we want.

    # If we're building under env, use a relative path for the link; otherwise use an absolute one.
    if str_contains "$OSG_BUILD_DIR" "$ENV_DIR"; then
        current_pwd=`pwd`
        rel_path=$(get_relative_path "$current_pwd" "$OSG_BUILD_DIR" )
        create_exist_symlink "$rel_path" openscenegraph
    else
        create_exist_symlink "$OSG_BUILD_DIR" openscenegraph  # this links directly to the most recent build...
    fi

    popd > /dev/null

    # Header Files
    pushd "$ENV_DIR/include" > /dev/null
    if [ ! -d "openscenegraph" ]; then  # We may have built here, so only create a symlink if not
        rm openscenegraph 2>/dev/null
        create_exist_symlink "$OSG_INSTALL_PREFIX/include/openscenegraph" openscenegraph
    fi
    popd > /dev/null

}

function main()
{

    ensure_install_prefix_writable $OSG_INSTALL_PREFIX

    #@todo: uncomment this (after fixing the check)
    if check_package_installed "$OSG_INSTALL_PREFIX/lib/libosg*"; then
        echo "- OpenSceneGraph is installed under prefix $OSG_INSTALL_PREFIX -- skipping."
        env_link_openscenegraph
        return
    fi

    if check_openscenegraph_built; then
        echo "- OpenSceneGraph is already built under $OSG_BUILD_DIR -- skipping."
        ensure_openscenegraph_openglsupport
        install_openscenegraph
        env_link_openscenegraph
        return
    fi
    
    if check_file_exists "$OSG_PACKAGE_DIR/CMakeLists.txt"; then
        echo "- OpenSceneGraph is already unpacked to $OSG_BUILD_DIR -- skipping."
        #patch_openscenegraph
        build_openscenegraph
        install_openscenegraph
        env_link_openscenegraph
        return
    fi

    if check_file_exists "$DOWNLOADS_DIR/$openscenegraph_pkg"; then
        echo "- OpenSceneGraph package already exists under env/downloads -- skipping download."
        unpack_openscenegraph
        #patch_openscenegraph
        build_openscenegraph
        install_openscenegraph
        env_link_openscenegraph
        return
    fi

    # If we haven't returned by now, we have to do everything
    download_openscenegraph
    unpack_openscenegraph
    #patch_openscenegraph
    build_openscenegraph
    install_openscenegraph
    env_link_openscenegraph

}


main
