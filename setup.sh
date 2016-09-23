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

# Purpose: Install script for everything related to the tensegrity repository
# Author:  Ryan Adams, Perry Bhandal

##############################################################################
#                         START DO NOT MODIFY                                #
##############################################################################
SCRIPT_PATH="`dirname \"$0\"`"
SCRIPT_PATH="`( cd \"$SCRIPT_PATH\" && pwd )`"
##############################################################################
#                          END DO NOT MODIFY                                 #
##############################################################################

# Add the relative path from this script to the helpers folder.
pushd "${SCRIPT_PATH}/bin/setup/helpers/" > /dev/null

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

CONF_FILES=("general.conf" "boost.conf" "bullet.conf" "build.conf" "jsoncpp.conf" "gmocktest.conf" "neuralnet.conf" "yamlcpp.conf")

function banner() 
{
    echo ""
    echo "== Setup ============"
}

function init_config()
{

    if [ -f "$CONF_DIR/install.conf" ]; then
        echo "Your conf directory contains install.conf, which has been deprecated. Please delete install.conf and run set up again. See issue 21 for more details."
        exit 1
    fi

    echo "- Starting configuration"

    ## Determine if any conf files are missing
    to_create=()

    for file_name in "${CONF_FILES[@]}"
    do
        if [ ! -f "$CONF_DIR/$file_name" ]; then
            to_create+=($file_name)
        fi
    done

    if [ "${#to_create}" -eq 0 ]; then
        source "$CONF_DIR/general.conf"
        return
    fi

    echo "You are missing one or more required conf files. Setup will now generate them."
    
    for file_name in "${to_create[@]}"
    do
        echo "Creating conf/$file_name"
        cp "$CONF_DIR/default/${file_name}.default" "$CONF_DIR/$file_name"  || { echo "Could not find default conf file ${file_name}.default in conf/default -- exiting now."; exit 1; }  
    done

    echo "=== CONF FILES CREATED ==="
    echo "**All missing configuration files have been created in conf/. Please edit as needed and then re-run setup.sh.**"

    exit 0
}


function init_scripts()
{
    # Make sure permissions are correct, etc.
    pushd "$SETUP_DIR" > /dev/null
    chmod a+x *
    popd > /dev/null
}

function run_setupscript()
{
    script_name=$1
    full_name=$2
    echo ""
    echo "Initializing ${2}..."
    "$SETUP_DIR/setup_${script_name}.sh" || { echo "$full_name initialization failed -- exiting now."; exit 1; }
}

# Set a variable in the install.conf configuration file
function set_config_var()
{
    var=$1
    value=$2
    echo "var: $var; value: $value"
    sed "s,$var=.*,$var=\"$value\",g" "$SETUP_DIR/install.conf" > "$SETUP_DIR/install.conf.tmp"
    mv "$SETUP_DIR/install.conf.tmp" "$SETUP_DIR/install.conf"

}

# EXECUTION
banner

init_config
init_scripts

set_multicore_make

run_setupscript "env" "Env directory"
run_setupscript "cmake" "CMake"
run_setupscript "gmocktest" "GMockTest"
run_setupscript "jsoncpp" "JsonCPP"
run_setupscript "boost" "Boost"
run_setupscript "yamlcpp" "YamlCPP"
run_setupscript "neuralnet" "Neural Net"
run_setupscript "bullet" "Bullet Physics Library"


echo ""
echo "Setup Complete!"
echo ""
echo "* Next Steps: Use bin/build.sh to build your code. The results will be placed under the 'build' directory."
echo ""
