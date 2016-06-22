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

# Purpose: Boost library Setup
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
source_conf "boost.conf"

# Variables
boost_pkg=`echo $BOOST_URL|awk -F/ '{print $NF}'`  # get the package name from the url

# Download the package to env/downloads
function download_boost()
{

    boost_pkg_path="$DOWNLOADS_DIR/$boost_pkg"

    if [ -f "$boost_pkg_path" ]; then
        echo "- Boost package already exists ('$boost_pkg_path') -- skipping download."
        return
    fi

    echo "Downloading $boost_pkg to $boost_pkg_path"
    download_file "$BOOST_URL" "$boost_pkg_path"
}

# Unpack to the build directory specified in install.conf
function unpack_boost()
{
    # Create directory and unpack
    if check_directory_exists "$BOOST_BUILD_DIR"; then
        echo "- Boost is already unpacked to '$BOOST_BUILD_DIR' -- skipping."
        return
    fi

    echo "Unpacking boost to $BOOST_BUILD_DIR (this may take a minute...)"
    # TODO: Do we need to remove the dir if it already exists?
    create_directory_if_noexist "$BOOST_BUILD_DIR"

    # Unzip
    pushd "$BOOST_BUILD_DIR" > /dev/null
    tar xf "$DOWNLOADS_DIR/$boost_pkg" --strip 1 
    popd > /dev/null

}

# Build the package under the build directory specified in in install.conf
function build_boost()
{

    echo "- Building Boost under $BOOST_BUILD_DIR"
    pushd "$BOOST_BUILD_DIR" > /dev/null

    if check_directory_exists "stage"; then
        echo "- Boost is already built in '$BOOST_BUILD_DIR' -- skipping."
        return
    fi

    #To make gcc as the default compiler, this line uncomments the line ';using gcc' in user-config.jam file of the boost library.
    sed -i 's/^# using gcc ;/using gcc ;/g' tools/build/v2/user-config.jam

    # Perform the build
    #./bootstrap.sh --prefix="$BOOST_INSTALL_PREFIX" --with-libraries=system || { echo "Boost bootstrap failed."; exit 1; } # Lite
    ./bootstrap.sh --prefix="$BOOST_INSTALL_PREFIX" || { echo "Boost bootstrap failed."; exit 1; }

    popd > /dev/null
}

# Install the package under the package install prefix from install.conf
function install_boost()
{

    echo "- Installing Boost under $BOOST_INSTALL_PREFIX"

    pushd "$BOOST_BUILD_DIR" > /dev/null
    ./b2 install || { echo "Install failed"; exit 1; }
    popd > /dev/null

}

# Create symlinks under env for building our applications and IDE integration
function env_link_boost()
{

    # Build
    pushd "$ENV_DIR/build" > /dev/null
    rm boost 2>/dev/null  # This will fail if 'boost' is a directory, which is what we want.
    if check_directory_exists "$BOOST_BUILD_DIR"; then  # If we built boost (as opposed to installing it another way)...

        # If we're building under env, use a relative link; otherwise use an absolute one.
        if str_contains "$BOOST_BUILD_DIR" "$ENV_DIR"; then
            current_pwd=`pwd`
            rel_path=$(get_relative_path "$current_pwd" "$BOOST_BUILD_DIR" )
            create_exist_symlink "$rel_path" boost
        else
            create_exist_symlink "$BOOST_BUILD_DIR" boost  # this links directly to the most recent build...
        fi

    fi
    popd > /dev/null

    # Header Files
    pushd "$ENV_DIR/include" > /dev/null
    if [ ! -d "boost" ]; then  # We may have built boost here, so only create a symlink if not
        rm boost 2>/dev/null
        create_exist_symlink "$BOOST_INSTALL_PREFIX/include/boost" boost
    fi
    popd > /dev/null

}

function main()
{

    ensure_install_prefix_writable $BOOST_INSTALL_PREFIX

    # TODO: Check to make sure that the header files are also installed properly.
    if check_package_installed "$BOOST_INSTALL_PREFIX/lib/libboost*"; then
        echo "- Boost is installed under prefix $BOOST_INSTALL_PREFIX -- skipping."
        env_link_boost
        return
    fi

    # Check if boost is built.
    if check_directory_exists "$BOOST_BUILD_DIR/stage"; then
        echo "- Boost is already built under $BOOST_BUILD_DIR -- skipping."
        install_boost
        env_link_boost
        return
    fi

    # Check if boost is unpacked
    if check_directory_exists "$BOOST_PACKAGE_DIR/boost"; then
        echo "- Boost is already unpacked to $BOOST_BUILD_DIR -- skipping."
        build_boost
        install_boost
        env_link_boost
        return
    fi

    # Check if boost is downloaded.
    if check_file_exists "$DOWNLOADS_DIR/$boost_pkg"; then
        echo "- Boost package already exists under env/downloads -- skipping download."
        unpack_boost
        build_boost
        install_boost
        env_link_boost
        return
    fi

    # If we haven't returned by now, we have to do everything
    download_boost
    unpack_boost
    build_boost
    install_boost
    env_link_boost

}


main
