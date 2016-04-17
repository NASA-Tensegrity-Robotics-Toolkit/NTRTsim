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

# Purpose: YamlCPP setup
# Date:    2015-12-12

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
source_conf "yamlcpp.conf"

# Variables
yamlcpp_pkg=`echo $YAMLCPP_URL|awk -F/ '{print $NF}'`  # get the package name from the url

# Check to see if yamlcpp has been built already
function check_yamlcpp_built()
{
    # Check for a library that's created when yamlcpp is built
    fname=$(find "$YAMLCPP_BUILD_DIR" -iname libyaml-cpp* 2>/dev/null)
    if [ -f "$fname" ]; then
        return $TRUE
    fi
    return $FALSE
}

# Download the package to env/downloads
function download_yamlcpp()
{

    yamlcpp_pkg_path="$DOWNLOADS_DIR/$yamlcpp_pkg"

    if [ -f "$yamlcpp_pkg_path" ]; then
        echo "- YamlCPP package already exists ('$yamlcpp_pkg_path') -- skipping download."
        return
    fi

    echo "Downloading $yamlcpp_pkg to $yamlcpp_pkg_path"
    download_file "$YAMLCPP_URL" "$yamlcpp_pkg_path"
}

# Unpack to the build directory specified in install.conf
function unpack_yamlcpp()
{
    # Create directory and unpack
    if check_directory_exists "$YAMLCPP_BUILD_DIR"; then
        echo "- YamlCPP is already unpacked to '$YAMLCPP_BUILD_DIR' -- skipping."
        return
    fi

    echo "Unpacking yamlcpp to $YAMLCPP_BUILD_DIR (this may take a minute...)"
    # TODO: Do we need to remove the dir if it already exists?
    create_directory_if_noexist $YAMLCPP_BUILD_DIR

    # Unzip
    pushd "$YAMLCPP_BUILD_DIR" > /dev/null
    unzip "$DOWNLOADS_DIR/$yamlcpp_pkg" || { echo "- ERROR: Failed to unpack YamlCPP"; exit 1; }
    mv yaml*/* .
    popd > /dev/null
}

# Build the package under the build directory specified in in install.conf
function build_yamlcpp()
{

    echo "- Building YamlCPP under $YAMLCPP_BUILD_DIR"
    pushd "$YAMLCPP_BUILD_DIR" > /dev/null

    # Perform the build
    # If you turn double precision on, turn it on in inc.CMakeYamlCPP.txt as well for the NTRT build
    "$ENV_DIR/bin/cmake" . -G "Unix Makefiles" \
        -DBUILD_SHARED_LIBS=OFF \
        -DBUILD_EXTRAS=ON \
        -DCMAKE_INSTALL_PREFIX="$YAMLCPP_INSTALL_PREFIX" \
        -DCMAKE_C_COMPILER="gcc" \
        -DCMAKE_CXX_COMPILER="g++" \
        -DCMAKE_C_FLAGS="-fPIC" \
        -DCMAKE_CXX_FLAGS="-fPIC" \
        -DCMAKE_EXE_LINKER_FLAGS="-fPIC" \
        -DCMAKE_MODULE_LINKER_FLAGS="-fPIC" \
        -DCMAKE_SHARED_LINKER_FLAGS="-fPIC" \
        -DUSE_DOUBLE_PRECISION=OFF \
        -DCMAKE_INSTALL_NAME_DIR="$YAMLCPP_INSTALL_PREFIX" || { echo "- ERROR: CMake for YamlCPP failed."; exit 1; }
    #If you turn this on, turn it on in inc.CMakeYamlCPP.txt as well for the NTRT build
    # Additional yamlcpp options:
    # -DFRAMEWORK=ON
    # -DBUILD_DEMOS=ON

    make || { echo "- ERROR: YamlCPP build failed"; exit 1; }

    popd > /dev/null
}

# Install the package under the package install prefix from install.conf
function install_yamlcpp()
{

    echo "- Installing YamlCPP under $YAMLCPP_INSTALL_PREFIX"

    pushd "$YAMLCPP_BUILD_DIR" > /dev/null

    make install || { echo "Install failed -- maybe you need to use sudo when running setup?"; exit 1; }

    popd > /dev/null
}

# Create symlinks under env for building our applications and IDE integration
function env_link_yamlcpp()
{

    # Build
    pushd "$ENV_DIR/build" > /dev/null
    rm yamlcpp 2>/dev/null   # Note: this will fail if 'yamlcpp' is a directory, which is what we want.

    # If we're building under env, use a relative path for the link; otherwise use an absolute one.
    if str_contains "$YAMLCPP_BUILD_DIR" "$ENV_DIR"; then
        current_pwd=`pwd`
        rel_path=$(get_relative_path "$current_pwd" "$YAMLCPP_BUILD_DIR" )
        create_exist_symlink "$rel_path" yamlcpp
    else
        create_exist_symlink "$YAMLCPP_BUILD_DIR" yamlcpp  # this links directly to the most recent build...
    fi

    popd > /dev/null

}

function main()
{

    ensure_install_prefix_writable $YAMLCPP_INSTALL_PREFIX

    if check_package_installed "$YAMLCPP_INSTALL_PREFIX/lib/libyaml-cpp*"; then
        echo "- YamlCPP is installed under prefix $YAMLCPP_INSTALL_PREFIX -- skipping."
        env_link_yamlcpp
        return
    fi

    if check_yamlcpp_built; then
        echo "- YamlCPP is already built under $YAMLCPP_BUILD_DIR -- skipping."
        install_yamlcpp
        env_link_yamlcpp
        return
    fi

    # @todo: add check yamlcpp patched

    if check_file_exists "$YAMLCPP_PACKAGE_DIR/CMakeLists.txt"; then
        echo "- YamlCPP is already unpacked to $YAMLCPP_BUILD_DIR -- skipping."
        build_yamlcpp
        install_yamlcpp
        env_link_yamlcpp
        return
    fi

    if check_file_exists "$DOWNLOADS_DIR/$yamlcpp_pkg"; then
        echo "- YamlCPP package already exists under env/downloads -- skipping download."
        unpack_yamlcpp
        build_yamlcpp
        install_yamlcpp
        env_link_yamlcpp
        return
    fi

    # If we haven't returned by now, we have to do everything
    download_yamlcpp
    unpack_yamlcpp
    build_yamlcpp
    install_yamlcpp
    env_link_yamlcpp

}


main
