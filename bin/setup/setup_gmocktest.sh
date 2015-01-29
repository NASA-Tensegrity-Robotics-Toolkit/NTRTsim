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

# Purpose: GMockTest setup
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
source_conf "gmocktest.conf"

# Variables
gmocktest_pkg=`echo $GMOCKTEST_URL|awk -F/ '{print $NF}'`  # get the package name from the url

# Check to see if gmocktest has been built already
function check_gmocktest_built()
{
    # Check for a library that's created when gmocktest is built   
    fname=$(find "$GMOCKTEST_BUILD_DIR" -iname libgmocktest* 2>/dev/null)
    if [ -f "$fname" ]; then
        return $TRUE
    fi
    return $FALSE
}

# Download the package to env/downloads
function download_gmocktest()
{

    gmocktest_pkg_path="$DOWNLOADS_DIR/$gmocktest_pkg"

    if [ -f "$gmocktest_pkg_path" ]; then
        echo "- GMockTest package already exists ('$gmocktest_pkg_path') -- skipping download."
        return
    fi

    echo "Downloading $gmocktest_pkg to $gmocktest_pkg_path"
    download_file "$GMOCKTEST_URL" "$gmocktest_pkg_path"
}

# Unpack to the build directory specified in install.conf
function unpack_gmocktest()
{
    # Create directory and unpack
    if check_directory_exists "$GMOCKTEST_BUILD_DIR"; then
        echo "- GMockTest is already unpacked to '$GMOCKTEST_BUILD_DIR' -- skipping."
        return
    fi

    echo "Unpacking gmocktest to $GMOCKTEST_BUILD_DIR (this may take a minute...)"
    # TODO: Do we need to remove the dir if it already exists?
    create_directory_if_noexist $GMOCKTEST_BUILD_DIR

    # Unzip
    pushd "$GMOCKTEST_BUILD_DIR" > /dev/null
    unzip "$DOWNLOADS_DIR/$gmocktest_pkg" || { echo "- ERROR: Failed to unpack GMockTest"; exit 1; }
    mv gmock*/* .
    popd > /dev/null
}

# Build the package under the build directory specified in in install.conf
function build_gmocktest()
{

    echo "- Building GMockTest under $GMOCKTEST_BUILD_DIR"
    pushd "$GMOCKTEST_BUILD_DIR" > /dev/null

    # Perform the build
    # If you turn double precision on, turn it on in inc.CMakeGMockTest.txt as well for the NTRT build
    "$ENV_DIR/bin/cmake" . -G "Unix Makefiles" \
        -DBUILD_SHARED_LIBS=OFF \
        -DBUILD_EXTRAS=ON \
        -DCMAKE_INSTALL_PREFIX="$GMOCKTEST_INSTALL_PREFIX" \
        -DCMAKE_C_FLAGS="-fPIC" \
        -DCMAKE_CXX_FLAGS="-fPIC" \
        -DCMAKE_EXE_LINKER_FLAGS="-fPIC" \
        -DCMAKE_MODULE_LINKER_FLAGS="-fPIC" \
        -DCMAKE_SHARED_LINKER_FLAGS="-fPIC" \
        -DUSE_DOUBLE_PRECISION=OFF \
        -DCMAKE_INSTALL_NAME_DIR="$GMOCKTEST_INSTALL_PREFIX" || { echo "- ERROR: CMake for GMockTest failed."; exit 1; }
    #If you turn this on, turn it on in inc.CMakeGMockTest.txt as well for the NTRT build
    # Additional gmocktest options: 
    # -DFRAMEWORK=ON
    # -DBUILD_DEMOS=ON

    make || { echo "- ERROR: GMockTest build failed"; exit 1; }

    popd > /dev/null
}

# Install the package under the package install prefix from install.conf
function install_gmocktest()
{

    echo "- Installing GMockTest under $GMOCKTEST_INSTALL_PREFIX"

    pushd "$GMOCKTEST_BUILD_DIR" > /dev/null

    create_exist_symlink $GMOCKTEST_BUILD_DIR/libgmock.a $LIB_DIR/
    create_exist_symlink $GMOCKTEST_BUILD_DIR/libgmock_main.a $LIB_DIR/

    create_exist_symlink $GMOCKTEST_BUILD_DIR/gtest/libgtest.a $LIB_DIR/
    create_exist_symlink $GMOCKTEST_BUILD_DIR/gtest/libgtest_main.a $LIB_DIR/

   popd > /dev/null
}

# Create symlinks under env for building our applications and IDE integration
function env_link_gmocktest()
{

    # Build
    pushd "$ENV_DIR/build" > /dev/null
    rm gmocktest 2>/dev/null   # Note: this will fail if 'gmocktest' is a directory, which is what we want.

    # If we're building under env, use a relative path for the link; otherwise use an absolute one.
    if str_contains "$GMOCKTEST_BUILD_DIR" "$ENV_DIR"; then
        current_pwd=`pwd`
        rel_path=$(get_relative_path "$current_pwd" "$GMOCKTEST_BUILD_DIR" )
        create_exist_symlink "$rel_path" gmocktest
    else
        create_exist_symlink "$GMOCKTEST_BUILD_DIR" gmocktest  # this links directly to the most recent build...
    fi

    # Symlink our header files in
    create_exist_symlink $GMOCKTEST_BUILD_DIR/gtest/include/gtest $INCLUDE_DIR/gtest
    create_exist_symlink $GMOCKTEST_BUILD_DIR/include/gmock $INCLUDE_DIR/gmock

    popd > /dev/null

}

function main()
{

    ensure_install_prefix_writable $GMOCKTEST_INSTALL_PREFIX

    if check_package_installed "$GMOCKTEST_INSTALL_PREFIX/lib/libgmock*"; then
        echo "- GMockTest is installed under prefix $GMOCKTEST_INSTALL_PREFIX -- skipping."
        env_link_gmocktest
        return
    fi

    if check_gmocktest_built; then
        echo "- GMockTest is already built under $GMOCKTEST_BUILD_DIR -- skipping."
        install_gmocktest
        env_link_gmocktest
        return
    fi

    if check_file_exists "$GMOCKTEST_PACKAGE_DIR/CMakeLists.txt"; then
        echo "- GMockTest is already unpacked to $GMOCKTEST_BUILD_DIR -- skipping."
        build_gmocktest
        install_gmocktest
        env_link_gmocktest
        return
    fi

    if check_file_exists "$DOWNLOADS_DIR/$gmocktest_pkg"; then
        echo "- GMockTest package already exists under env/downloads -- skipping download."
        unpack_gmocktest
        build_gmocktest
        install_gmocktest
        env_link_gmocktest
        return
    fi

    # If we haven't returned by now, we have to do everything
    download_gmocktest
    unpack_gmocktest
    build_gmocktest
    install_gmocktest
    env_link_gmocktest

}


main
