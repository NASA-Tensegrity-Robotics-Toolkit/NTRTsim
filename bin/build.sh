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

##############################################################################
#                         START DO NOT MODIFY                                #
##############################################################################
SCRIPT_PATH="`dirname \"$0\"`"
SCRIPT_PATH="`( cd \"$SCRIPT_PATH\" && pwd )`"
##############################################################################
#                          END DO NOT MODIFY                                 #
##############################################################################

# Add the relative path from this script to the helpers folder.
pushd "${SCRIPT_PATH}/setup/helpers/" > /dev/null

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

function usage
{
    echo "usage: $0 [-h] [-c] [-w] [-t/r/i/g] [build_path]"
    echo ""
    echo "positional arguments:"
    echo "  build_path            Path to build (relative to src, e.g. 'BasicApp' or"
    echo "                        'lib/Example'). Defaults to all if not specified."
    echo ""
    echo "optional arguments:"
    echo "  -h       Show this help message and exit"
    echo "  -c       Run 'make clean' before make/make install on non-library sources"
    echo "  -w       Show compiler warnings when building"
    echo "  -t       Build test/ rather than src/" 
    echo "  -r       Build test/ rather than src/ *and* run all tests after compilation."
    echo "  -i       Build test_integration/ rather than src/" 
    echo "  -g       Build test_integration/ rather than src/ *and* run all tests after compilation."
}

function cmake_cross_platform()
{
    "$ENV_BIN_DIR/cmake" $build_src \
        -G "$build_type" \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_INSTALL_PREFIX="$BASE_DIR/env" \
        -DCMAKE_INSTALL_NAME_DIR="$BASE_DIR/env" \
        -DCMAKE_CXX_FLAGS="$cmake_cxx_flags" \
        -DCMAKE_CXX_COMPILER="$ENV_BIN_DIR/g++" \
        -DCMAKE_C_FLAGS="-fPIC" \
        -DCMAKE_CXX_FLAGS="-fPIC" \
        -DCMAKE_EXE_LINKER_FLAGS="-fPIC" \
        -DCMAKE_MODULE_LINKER_FLAGS="-fPIC" \
        -DCMAKE_SHARED_LINKER_FLAGS="-fPIC" \
        || { echo "- ERROR: CMake for Bullet Physics failed."; exit 1; }
}

build_target=$BUILD_DIR
build_src=$SRC_DIR

# Handle Arguments
MAKE_CLEAN_FLAG=false
CMAKE_COMPILER_WARNINGS_FLAG=false
RUN_ALL_TESTS=false
RUN_INTEGRATION_TESTS=false

while getopts ":hcwtrig" opt; do
    case $opt in
        h)
            usage;
            exit 0;
            ;;
        c)
            MAKE_CLEAN_FLAG=true
            ;;
        w)
            CMAKE_COMPILER_WARNINGS_FLAG=true
            ;;
        t)
            build_target=$BUILD_TEST_DIR 
            build_src=$TEST_DIR
            ;;
        r)
            build_target=$BUILD_TEST_DIR 
            build_src=$TEST_DIR
            RUN_ALL_TESTS=true
            ;;
        i)
            build_target=$BUILD_INTEGRATION_TEST_DIR 
            build_src=$INTEGRATION_TEST_DIR
            ;;
        g)
            build_target=$BUILD_INTEGRATION_TEST_DIR 
            build_src=$INTEGRATION_TEST_DIR
            RUN_INTEGRATION_TESTS=true
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


# Make sure the build directory exists
create_directory_if_noexist $build_target
if [ ! -d "$build_target" ]; then
    echo "Unable to create build directory '$build_target' -- exiting."
    exit 2
fi


TO_BUILD=${@:$OPTIND:1}

if [ "$TO_BUILD" != "" ]; then
    echo "Building src/$TO_BUILD => build/$TO_BUILD";
else
    echo "Building src/ => build/";
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
pushd "$build_target" > /dev/null

# Uncomment this to create standard unix makefiles
build_type="Unix Makefiles"
# Uncomment this to create unix makefiles as well as the eclipse CDT4 project files
#build_type="Eclipse CDT4 - Unix Makefiles"

if $CMAKE_COMPILER_WARNINGS_FLAG; then
    cmake_cxx_flags="-Wall -Wno-long-long"
else
    cmake_cxx_flags=""
fi

cmake_cross_platform

popd > /dev/null # exit build dir (done with cmake)

# Make / install
if [ "$TO_BUILD" != "" ]; then
    pushd "$build_target/$TO_BUILD" > /dev/null
else
    pushd "$build_target" > /dev/null
fi    

# Make clean if requested
if $MAKE_CLEAN_FLAG; then
    make clean
fi

make || exit 1

popd > /dev/null  # exit make dir

popd > /dev/null  # exit base dir

# Run all tests if necessary
if $RUN_ALL_TESTS; then
    if [ ! -d $BUILD_TEST_DIR ]; then
        echo "Build test directory does not exist. Have the tests been compiled?"
        exit 1
    fi
	
	pushd $BUILD_TEST_DIR > /dev/null
fi

if $RUN_INTEGRATION_TESTS; then
	if [ ! -d $BUILD_INTEGRATION_TEST_DIR ]; then
        echo "Build integration test directory does not exist. Have the tests been compiled?"
        exit 1
    fi
	
	pushd $BUILD_INTEGRATION_TEST_DIR > /dev/null
fi

if $RUN_ALL_TESTS || $RUN_INTEGRATION_TESTS; then
    if ! has_command "python"; then
        echo "=== MISSING DEPENDENCY ==="
        echo "Python 2.7 is required for automated test running. You don't appear to have it installed."
        exit 1
    fi

    python ${SHELL_UTILITIES_DIR}/runAllTests.py || { 
        echo ""
        echo "=== TEST FAILURE(S) ==="
        echo ""
        echo "One or more tests have failed!"
        echo "Search the console output for 'FAILED' to find each failing test."
        echo "Or search for 'FAILED TEST' to find each failing test group."
        exit 1
    }

    echo ""
    echo "*** All tests succeeded! ***"
    echo ""

    popd > /dev/null
fi
