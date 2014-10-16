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

# Purpose: Setup for python virtualenv (without root access)
# Date:    2013-09-29

##############################################################################
#                         START DO NOT MODIFY                                #
##############################################################################
SCRIPT_PATH="`dirname \"$0\"`"
SCRIPT_PATH="`( cd \"$SCRIPT_PATH\" && pwd )`"

# Add the relative path from this script to the helpers folder.
pushd "${SCRIPT_PATH}/helpers/" > /dev/null

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
source_conf "virtualenv.conf"


# Variables
virtualenv_pkg=`echo $VENV_URL|awk -F/ '{print $NF}'`  # get the package name from the url

# Download the package to env/downloads
function download_virtualenv()
{
    virtualenv_pkg_path="$DOWNLOADS_DIR/$virtualenv_pkg"

    if [ -f "$virtualenv_pkg_path" ]; then
        echo "- Virtualenv package already exists ('$virtualenv_pkg_path') -- skipping download."
        return
    fi

    echo "Downloading $virtualenv_pkg to $virtualenv_pkg_path"
    download_file "$VENV_URL" "$virtualenv_pkg_path"
}

# Unpack to the build directory specified in the appropriate .conf file
function unpack_virtualenv()
{
    # Create directory and unpack
    if check_directory_exists "$VENV_BUILD_DIR"; then
        echo "- Virtualenv is already unpacked to '$VENV_BUILD_DIR' -- skipping."
        return
    fi

    echo "Unpacking virtualenv to $VENV_BUILD_DIR (this may take a minute...)"
    # TODO: Do we need to remove the dir if it already exists?
    create_directory_if_noexist "$VENV_BUILD_DIR"

    # Unzip
    pushd "$VENV_BUILD_DIR" > /dev/null
    tar xf "$DOWNLOADS_DIR/$virtualenv_pkg" --strip 1 
    popd > /dev/null

}

# Create a local virtualenv (note: this does not require sudo access)
function install_virtualenv()
{

    if [ -d "$VENV_DIR" ]; then
        echo "- Virtualenv has already been created ('$VENV_DIR') -- skipping install."
        echo "  NOTE: If you want to have setup re-create the virtualenv, delete $VENV_DIR and re-run setup.sh"
        return
    fi
    
    echo "- Creating new virtualenv under $VENV_INSTALL_PREFIX"
    
    pushd "$VENV_BUILD_DIR" > /dev/null
    "$VENV_PYTHON_INTERPRETER" "virtualenv.py" "$VENV_DIR"
    popd > /dev/null
}


function main()
{

    ensure_install_prefix_writable $VENV_INSTALL_PREFIX

    # Create the virtualenv
    download_virtualenv
    unpack_virtualenv
    install_virtualenv

}


main
