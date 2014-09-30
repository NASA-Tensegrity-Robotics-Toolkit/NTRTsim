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


# Call virtualenv.py with --no-setuptools to create a new virtualenv
python "$SETUP_DIR/helpers/virtualenv.py" --no-setuptools "$ENV_DIR/venv"

# Activate the new virtualenv
source "$ENV_DIR/venv/bin/activate"

# Install pip
easy_install pip

# Anything else?

# Deactivate the virtualenv
deactivate


