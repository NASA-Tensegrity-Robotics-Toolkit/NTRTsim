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

# Purpose: Bullet Physics setup
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
source_conf "bullet.conf"

# Variables
bullet_pkg=`echo $BULLET_URL|awk -F/ '{print $NF}'`  # get the package name from the url

# Check to see if bullet has been built already
function check_bullet_built()
{
    # Check for a library that's created when bullet is built   
    fname=$(find "$BULLET_BUILD_DIR" -iname libBulletCollision.* 2>/dev/null)
    if [ -f "$fname" ]; then
        return $TRUE
    fi
    return $FALSE
}

function ensure_bullet_openglsupport()
{
    result=$(count_files "$BULLET_BUILD_DIR/Demos/OpenGL/libOpenGLSupport.*")
    if [ "$result" == "0" ]; then
        echo "ERROR: It seems that bullet has been installed under prefix $BULLET_INSTALL_PREFIX, \
            but libOpenGLSupport was not found under the BULLET_BUILD_DIR ($BULLET_BUILD_DIR/Demos/OpenGL)."
        echo "  Option 1: Make sure that BULLET_BUILD_DIR in install.conf points to the location where you built bullet."
        echo "  Option 2: Change BULLET_INSTALL_PREFIX in install.conf to something besides $BULLET_INSTALL_PREFIX (see install.conf for more details)."
        echo "  Option 3: Remove the bullet files from the existing install location to have setup build bullet for you. To do this:"
        echo "            - remove the $BULLET_INSTALL_PREFIX/include/bullet directory"
        echo "            - remove the bullet libraries ($BULLET_INSTALL_PREFIX/lib/libBullet*, $BULLET_INSTALL_PREFIX/lib/libLinearMath.*, and $BULLET_INSTALL_PREFIX/lib/libMiniCL.*)"
        exit 1
    fi
}

# Download the package to env/downloads
function download_bullet()
{

    bullet_pkg_path="$DOWNLOADS_DIR/$bullet_pkg"

    if [ -f "$bullet_pkg_path" ]; then
        echo "- Bullet Physics package already exists ('$bullet_pkg_path') -- skipping download."
        return
    fi

    echo "Downloading $bullet_pkg to $bullet_pkg_path"
    download_file "$BULLET_URL" "$bullet_pkg_path"
}

# Unpack to the build directory specified in install.conf
function unpack_bullet()
{
    # Create directory and unpack
    if check_directory_exists "$BULLET_BUILD_DIR"; then
        echo "- Bullet Physics is already unpacked to '$BULLET_BUILD_DIR' -- skipping."
        return
    fi

    echo "Unpacking bullet to $BULLET_BUILD_DIR (this may take a minute...)"
    # TODO: Do we need to remove the dir if it already exists?
    create_directory_if_noexist $BULLET_BUILD_DIR

    # Unzip
    pushd "$BULLET_BUILD_DIR" > /dev/null
    tar xf "$DOWNLOADS_DIR/$bullet_pkg" --strip 1 || { echo "- ERROR: Failed to unpack Bullet Physics."; exit 1; }
    popd > /dev/null
}

# Patch Bullet to include OpenGL Directories
function patch_bullet()
{
    pushd "$BULLET_BUILD_DIR/Demos" > /dev/null

    # Copy the files we're going to change
    create_directory_if_noexist "OpenGL_FreeGlut"
    cp "OpenGL/CMakeLists.txt" "OpenGL_FreeGlut/CMakeLists.txt"
    cp "OpenGL/DemoApplication.h" "OpenGL_FreeGlut/tgDemoApplication.h"
    cp "OpenGL/DemoApplication.cpp" "OpenGL_FreeGlut/tgDemoApplication.cpp"
    cp "OpenGL/GLDebugDrawer.h" "OpenGL_FreeGlut/tgGLDebugDrawer.h"
    cp "OpenGL/GLDebugDrawer.cpp" "OpenGL_FreeGlut/tgGLDebugDrawer.cpp"
    cp "OpenGL/GlutDemoApplication.h" "OpenGL_FreeGlut/tgGlutDemoApplication.h"
    cp "OpenGL/GlutDemoApplication.cpp" "OpenGL_FreeGlut/tgGlutDemoApplication.cpp"
    cp "OpenGL/GlutStuff.h" "OpenGL_FreeGlut/tgGlutStuff.h"
    cp "OpenGL/GlutStuff.cpp" "OpenGL_FreeGlut/tgGlutStuff.cpp"

    # Patch them
    patch -p5 < "$SETUP_DIR/patches/CMakePatch.diff"
    patch -p5 < "$SETUP_DIR/patches/OpenGLPatch.diff"

    popd > /dev/null
    
    # Also fix double free error in btQuickprof
    
    pushd "$BULLET_BUILD_DIR/src/LinearMath" > /dev/null
    
    patch < "$SETUP_DIR/patches/btQuickprof.patch"
    
    popd > /dev/null
}

# Build the package under the build directory specified in in install.conf
function build_bullet()
{

    echo "- Building Bullet Physics under $BULLET_BUILD_DIR"
    pushd "$BULLET_BUILD_DIR" > /dev/null

    # Perform the build
    # If you turn double precision on, turn it on in inc.CMakeBullet.txt as well for the NTRT build
    "$ENV_DIR/bin/cmake" . -G "Unix Makefiles" \
        -DBUILD_SHARED_LIBS=OFF \
        -DBUILD_EXTRAS=ON \
        -DCMAKE_INSTALL_PREFIX="$BULLET_INSTALL_PREFIX" \
        -DCMAKE_C_FLAGS="-fPIC" \
        -DCMAKE_CXX_FLAGS="-fPIC" \
        -DCMAKE_C_COMPILER="gcc" \
        -DCMAKE_CXX_COMPILER="g++" \
        -DCMAKE_EXE_LINKER_FLAGS="-fPIC" \
        -DCMAKE_MODULE_LINKER_FLAGS="-fPIC" \
        -DCMAKE_SHARED_LINKER_FLAGS="-fPIC" \
        -DUSE_DOUBLE_PRECISION=ON \
        -DCMAKE_INSTALL_NAME_DIR="$BULLET_INSTALL_PREFIX" || { echo "- ERROR: CMake for Bullet Physics failed."; exit 1; }
    #If you turn this on, turn it on in inc.CMakeBullet.txt as well for the NTRT build
    # Additional bullet options: 
    # -DFRAMEWORK=ON
    # -DBUILD_DEMOS=ON

    make || { echo "- ERROR: Bullet build failed. Attempting to explicitly make from directory."; make_bullet_local; }

    popd > /dev/null
}

function make_bullet_local()
{
    pushd "$BULLET_BUILD_DIR" > /dev/null

    make || { echo "Explicit make of Bullet failed as well."; exit 1; }

    popd > /dev/null
}

# Install the package under the package install prefix from install.conf
function install_bullet()
{

    echo "- Installing Bullet Physics under $BULLET_INSTALL_PREFIX"

    pushd "$BULLET_BUILD_DIR" > /dev/null

    make install || { echo "Install failed -- maybe you need to use sudo when running setup?"; exit 1; }

    popd > /dev/null
}

# Create symlinks under env for building our applications and IDE integration
function env_link_bullet()
{

    # Build
    pushd "$ENV_DIR/build" > /dev/null
    rm bullet 2>/dev/null   # Note: this will fail if 'bullet' is a directory, which is what we want.

    # If we're building under env, use a relative path for the link; otherwise use an absolute one.
    if str_contains "$BULLET_BUILD_DIR" "$ENV_DIR"; then
        current_pwd=`pwd`
        rel_path=$(get_relative_path "$current_pwd" "$BULLET_BUILD_DIR" )
        create_exist_symlink "$rel_path" bullet
    else
        create_exist_symlink "$BULLET_BUILD_DIR" bullet  # this links directly to the most recent build...
    fi

    popd > /dev/null

    # Header Files
    pushd "$ENV_DIR/include" > /dev/null
    if [ ! -d "bullet" ]; then  # We may have built here, so only create a symlink if not
        rm bullet 2>/dev/null
        create_exist_symlink "$BULLET_INSTALL_PREFIX/include/bullet" bullet
    fi
    popd > /dev/null

}

function main()
{

    ensure_install_prefix_writable $BULLET_INSTALL_PREFIX

    if check_package_installed "$BULLET_INSTALL_PREFIX/lib/libBulletDynamics*"; then
        echo "- Bullet Physics is installed under prefix $BULLET_INSTALL_PREFIX -- skipping."
        ensure_bullet_openglsupport
        env_link_bullet
        return
    fi

    if check_bullet_built; then
        echo "- Bullet Physics is already built under $BULLET_BUILD_DIR -- skipping."
        ensure_bullet_openglsupport
        install_bullet
        env_link_bullet
        return
    fi
    
    # This may not be the best test - The directory is created at the beginning of the function
    # Is there a way to check a specific line into a file?
    if check_directory_exists "$BULLET_BUILD_DIR/Demos/OpenGL_FreeGlut/"; then
        echo "- Bullet Physics patches have already been applied -- skipping."
        build_bullet
        install_bullet
        env_link_bullet
        return
    fi

    if check_file_exists "$BULLET_PACKAGE_DIR/CMakeLists.txt"; then
        echo "- Bullet Physics is already unpacked to $BULLET_BUILD_DIR -- skipping."
        patch_bullet
        build_bullet
        install_bullet
        env_link_bullet
        return
    fi

    if check_file_exists "$DOWNLOADS_DIR/$bullet_pkg"; then
        echo "- Bullet Physics package already exists under env/downloads -- skipping download."
        unpack_bullet
        patch_bullet
        build_bullet
        install_bullet
        env_link_bullet
        return
    fi

    # If we haven't returned by now, we have to do everything
    download_bullet
    unpack_bullet
    patch_bullet
    build_bullet
    install_bullet
    env_link_bullet

}


main
