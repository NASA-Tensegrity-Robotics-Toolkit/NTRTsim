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

# Purpose: Build the source tree including libraries and applications.
# Author:  Ryan Adams, Perry Bhandal
# Date:    July 2014
# Notes:   This is intended to be run any time you need to build the 
#          source tree.

# Get the script path so we can run this from anywhere
SCRIPT_PATH="`dirname \"$0\"`"                  # relative
BASE_DIR="`( cd \"$SCRIPT_PATH/..\" && pwd )`"  # absolutized and normalized
SRC_DIR="`( cd \"$BASE_DIR/src\" && pwd )`"
BUILD_DIR="$BASE_DIR/build"

function usage
{
    echo "usage: $0 [-h] [-c] [-l] [-s] [-w] [build_path]"
    echo ""
    echo "positional arguments:"
    echo "  build_path            Path to build (relative to src, e.g. 'BasicApp' or"
    echo "                        'lib/Example'). Defaults to all if not specified."
    echo ""
    echo "optional arguments:"
    echo "  -h       Show this help message and exit"
    echo "  -c       Run 'make clean' before make/make install on non-library sources"
    echo "  -l       Run 'make clean' before make/make install on libraries"
    echo "  -s       Don't automatically build the libraries"
    echo "  -w       Show compiler warnings when building"
}

# Since OS X Mavericks places the g++ compiler in a different place than
# Linux, and since we want CMake to automatically find g++ on linux distros,
# run one of two possible functions for actually compiling.
# Functions for calling CMake, depending on operating system
function cmake_OSX()
{
    "$BASE_DIR/env/bin/cmake" ../src \
        -G "$build_type" \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_INSTALL_PREFIX="$BASE_DIR/env" \
        -DCMAKE_INSTALL_NAME_DIR="$BASE_DIR/env" \
        -DCMAKE_CXX_FLAGS="$cmake_cxx_flags" \
        -DCMAKE_CXX_COMPILER="g++" \
        -DCMAKE_C_FLAGS="-fPIC" \
        -DCMAKE_CXX_FLAGS="-fPIC" \
        -DCMAKE_EXE_LINKER_FLAGS="-fPIC" \
        -DCMAKE_MODULE_LINKER_FLAGS="-fPIC" \
        -DCMAKE_SHARED_LINKER_FLAGS="-fPIC" \
        || { echo "- ERROR: CMake for Bullet Physics failed."; exit 1; }
}

function cmake_linux()
{
    "$BASE_DIR/env/bin/cmake" ../src \
        -G "$build_type" \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_INSTALL_PREFIX="$BASE_DIR/env" \
        -DCMAKE_INSTALL_NAME_DIR="$BASE_DIR/env" \
        -DCMAKE_CXX_FLAGS="$cmake_cxx_flags" \
        -DCMAKE_C_FLAGS="-fPIC" \
        -DCMAKE_CXX_FLAGS="-fPIC" \
        -DCMAKE_EXE_LINKER_FLAGS="-fPIC" \
        -DCMAKE_MODULE_LINKER_FLAGS="-fPIC" \
        -DCMAKE_SHARED_LINKER_FLAGS="-fPIC" \
        || { echo "- ERROR: CMake for Bullet Physics failed."; exit 1; }

}

# Make sure the build directory exists
if [ ! -d "$BUILD_DIR" ]; then
    mkdir "$BUILD_DIR"
fi
if [ ! -d "$BUILD_DIR" ]; then
    echo "Unable to create build directory '$BUILD_DIR' -- exiting."
    exit 2
fi

# Handle Arguments
MAKE_CLEAN_FLAG=false
MAKE_CLEAN_LIB_FLAG=false
MAKE_LIB_FLAG=true
CMAKE_COMPILER_WARNINGS_FLAG=false

while getopts ":hclsw" opt; do
    case $opt in
        h)
            usage;
            exit 0;
            ;;
        c)
            MAKE_CLEAN_FLAG=true
            ;;
        l)
            MAKE_CLEAN_LIB_FLAG=true
            ;;
        s)
            MAKE_LIB_FLAG=false
            ;;
        w)
            CMAKE_COMPILER_WARNINGS_FLAG=true
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            exit 1
            ;;
        :)
            echo "Option -$OPTARG requires an argument." >&2
            exit 1
            ;;
    esac
done

TO_BUILD=${@:$OPTIND:1}

if [ "$TO_BUILD" != "" ]; then
    echo "Building src/$TO_BUILD => build/$TO_BUILD";
else
    echo "Building src/ => build/";
    if $MAKE_CLEAN_FLAG; then
        MAKE_LIB_FLAG=true  # have to make the libs if we're doing a full clean
        MAKE_CLEAN_LIB_FLAG=true
    fi
fi

if [ ! -d "$SRC_DIR/$TO_BUILD" ]; then
    echo "Requested build target '$SRC_DIR/$TO_BUILD' not found -- exiting."
    exit 2
fi

# Go to the base directory
pushd "$BASE_DIR" > /dev/null

# Make sure setup has been run
if [ ! -d env ]; then
    echo "Setup must be run before building -- see README.txt for more info."
    exit 1
fi

### COMPILING STEP

# CMake everything (@todo: can we just have it cmake certain things?)
pushd "$BUILD_DIR" > /dev/null

# Uncomment this to create standard unix makefiles
build_type="Unix Makefiles"
# Uncomment this to create unix makefiles as well as the eclipse CDT4 project files
#build_type="Eclipse CDT4 - Unix Makefiles"

if $CMAKE_COMPILER_WARNINGS_FLAG; then
    cmake_cxx_flags="-Wall -Wno-long-long"
else
    cmake_cxx_flags=""
fi


if [ $(uname) == 'Darwin' ]
then
    cmake_OSX
else
    cmake_linux
fi


popd > /dev/null # exit build dir (done with cmake)

# Make / install
if [ "$TO_BUILD" != "" ]; then
    pushd "$BUILD_DIR/$TO_BUILD" > /dev/null
else
    pushd "$BUILD_DIR" > /dev/null
fi    

# Make clean if requested
if $MAKE_CLEAN_FLAG; then
    make clean
fi

make || exit 1

popd > /dev/null  # exit make dir

popd > /dev/null  # exit base dir
