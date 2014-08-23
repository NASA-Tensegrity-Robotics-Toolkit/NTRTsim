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

# Purpose: Env setup
# Date:    2013-05-04

##############################################################################
#                       Services Configuration                               #
##############################################################################
# Add the relative path from your current directory to the bash services folder
# so we can import all helper scripts. If this script is operating from the
# root directory
SCRIPT_PATH="`dirname \"$0\"`"                  # relative
SCRIPT_PATH="`( cd \"$SCRIPT_PATH\" && pwd )`"  # absolutized and normalized

pushd "${SCRIPT_PATH}/../../services/bash/" > /dev/null

if [ ! -f "helper_functions.sh" ]; then
    echo "Could not find helper_functions.sh. Are we in the bash services folder?"
    exit 1;
fi

# Import our common files
source "helper_functions.sh"
source "helper_paths.sh"
source "helper_definitions.sh"

# Get out of the bash services folder.
popd > /dev/null
##############################################################################

function get_actual_user()
{
    who am i | awk '{print $1}'
}

function get_primary_group()
{
    id -g -n $1
}

if [ -d "$ENV_DIR" ]; then
    echo "- env directory exists. Ensuring subdirectories."
else
    mkdir "$ENV_DIR"
fi    
pushd "$ENV_DIR" > /dev/null
mkdir bin build downloads include lib 2>/dev/null
popd > /dev/null


# Permissions (change the env dir to be owned by the real current user)
# @todo: do we need to use this? We may not need sudo now...
actual_user=$(get_actual_user)
primary_group=$(get_primary_group $actual_user)
echo "- Changing ownership of env to current user ($actual_user:$primary_group)"
# Test for sudo (try a non-recursive change for speed)
chown $actual_user:$primary_group "$ENV_DIR" 2>/dev/null
if [ ! $? -eq 0 ]; then
    echo "  - ERROR: sudo required -- please re-run the command with sudo."
    exit 1;
fi
# Actually change the permissions
chown -R -P $actual_user:$primary_group "$ENV_DIR"
