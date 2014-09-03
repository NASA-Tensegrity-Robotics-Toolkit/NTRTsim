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

# Purpose: JsonCPP setup
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
source_conf "jsoncpp.conf"

# Variables
jsoncpp_pkg=`echo $JSONCPP_URL|awk -F/ '{print $NF}'`  # get the package name from the url

# Check to see if jsoncpp has been built already
function check_jsoncpp_built()
{
    # Check for a library that's created when jsoncpp is built   
    fname=$(find "$JSONCPP_BUILD_DIR" -iname libjsoncpp* 2>/dev/null)
    if [ -f "$fname" ]; then
        return $TRUE
    fi
    return $FALSE
}

# Download the package to env/downloads
function download_jsoncpp()
{

    jsoncpp_pkg_path="$DOWNLOADS_DIR/$jsoncpp_pkg"

    if [ -f "$jsoncpp_pkg_path" ]; then
        echo "- JsonCPP package already exists ('$jsoncpp_pkg_path') -- skipping download."
        return
    fi

    echo "Downloading $jsoncpp_pkg to $jsoncpp_pkg_path"
    download_file "$JSONCPP_URL" "$jsoncpp_pkg_path"
}

# Unpack to the build directory specified in install.conf
function unpack_jsoncpp()
{
    # Create directory and unpack
    if check_directory_exists "$JSONCPP_BUILD_DIR"; then
        echo "- JsonCPP is already unpacked to '$JSONCPP_BUILD_DIR' -- skipping."
        return
    fi

    echo "Unpacking jsoncpp to $JSONCPP_BUILD_DIR (this may take a minute...)"
    # TODO: Do we need to remove the dir if it already exists?
    create_directory_if_noexist $JSONCPP_BUILD_DIR

    # Unzip
    pushd "$JSONCPP_BUILD_DIR" > /dev/null
    unzip "$DOWNLOADS_DIR/$jsoncpp_pkg" || { echo "- ERROR: Failed to unpack JsonCPP"; exit 1; }
    mv json*/* .
    popd > /dev/null
}

# Build the package under the build directory specified in in install.conf
function build_jsoncpp()
{

    echo "- Building JsonCPP under $JSONCPP_BUILD_DIR"
    pushd "$JSONCPP_BUILD_DIR" > /dev/null

    # Perform the build
    # If you turn double precision on, turn it on in inc.CMakeJsonCPP.txt as well for the NTRT build
    "$ENV_DIR/bin/cmake" . -G "Unix Makefiles" \
        -DBUILD_SHARED_LIBS=OFF \
        -DBUILD_EXTRAS=ON \
        -DCMAKE_INSTALL_PREFIX="$JSONCPP_INSTALL_PREFIX" \
        -DCMAKE_C_COMPILER="gcc" \
        -DCMAKE_CXX_COMPILER="g++" \
        -DCMAKE_C_FLAGS="-fPIC" \
        -DCMAKE_CXX_FLAGS="-fPIC" \
        -DCMAKE_EXE_LINKER_FLAGS="-fPIC" \
        -DCMAKE_MODULE_LINKER_FLAGS="-fPIC" \
        -DCMAKE_SHARED_LINKER_FLAGS="-fPIC" \
        -DUSE_DOUBLE_PRECISION=OFF \
        -DCMAKE_INSTALL_NAME_DIR="$JSONCPP_INSTALL_PREFIX" || { echo "- ERROR: CMake for JsonCPP failed."; exit 1; }
    #If you turn this on, turn it on in inc.CMakeJsonCPP.txt as well for the NTRT build
    # Additional jsoncpp options: 
    # -DFRAMEWORK=ON
    # -DBUILD_DEMOS=ON

    make || { echo "- ERROR: JsonCPP build failed"; exit 1; }

    popd > /dev/null
}

# Install the package under the package install prefix from install.conf
function install_jsoncpp()
{

    echo "- Installing JsonCPP under $JSONCPP_INSTALL_PREFIX"

    pushd "$JSONCPP_BUILD_DIR" > /dev/null

    make install || { echo "Install failed -- maybe you need to use sudo when running setup?"; exit 1; }

    popd > /dev/null
}

# Create symlinks under env for building our applications and IDE integration
function env_link_jsoncpp()
{

    # Build
    pushd "$ENV_DIR/build" > /dev/null
    rm jsoncpp 2>/dev/null   # Note: this will fail if 'jsoncpp' is a directory, which is what we want.

    # If we're building under env, use a relative path for the link; otherwise use an absolute one.
    if str_contains "$JSONCPP_BUILD_DIR" "$ENV_DIR"; then
        current_pwd=`pwd`
        rel_path=$(get_relative_path "$current_pwd" "$JSONCPP_BUILD_DIR" )
        create_exist_symlink "$rel_path" jsoncpp
    else
        create_exist_symlink "$JSONCPP_BUILD_DIR" jsoncpp  # this links directly to the most recent build...
    fi

    popd > /dev/null

}

function main()
{

    ensure_install_prefix_writable $JSONCPP_INSTALL_PREFIX

    if check_package_installed "$JSONCPP_INSTALL_PREFIX/lib/libjsoncpp*"; then
        echo "- JsonCPP is installed under prefix $JSONCPP_INSTALL_PREFIX -- skipping."
        env_link_jsoncpp
        return
    fi

    if check_jsoncpp_built; then
        echo "- JsonCPP is already built under $JSONCPP_BUILD_DIR -- skipping."
        install_jsoncpp
        env_link_jsoncpp
        return
    fi

    # @todo: add check jsoncpp patched

    if check_file_exists "$JSONCPP_PACKAGE_DIR/CMakeLists.txt"; then
        echo "- JsonCPP is already unpacked to $JSONCPP_BUILD_DIR -- skipping."
        build_jsoncpp
        install_jsoncpp
        env_link_jsoncpp
        return
    fi

    if check_file_exists "$DOWNLOADS_DIR/$jsoncpp_pkg"; then
        echo "- JsonCPP package already exists under env/downloads -- skipping download."
        unpack_jsoncpp
        build_jsoncpp
        install_jsoncpp
        env_link_jsoncpp
        return
    fi

    # If we haven't returned by now, we have to do everything
    download_jsoncpp
    unpack_jsoncpp
    build_jsoncpp
    install_jsoncpp
    env_link_jsoncpp

}


main
