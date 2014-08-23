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

helper_pwd=`pwd`
SETUP_DIR="${helper_pwd}/../../setup/"
CONF_DIR="${helper_pwd}/../../conf/"
BASE_DIR="${helper_pwd}/../../"

# TODO: Remove this. It's a kludge until we can clean up the conf files and
# remove their dependence on base_dir rather than BASE_DIR.
base_dir=$BASE_DIR

# We put this as a function rather than simply declaring it because when
# the user imports this file, they'll have done pushd to get into bash
# services. They should call this after they've run popd following
# their imports. 
function getScriptPath()
{
    SCRIPT_PATH="`dirname \"$0\"`"                  # relative
    SCRIPT_PATH="`( cd \"$SCRIPT_PATH\" && pwd )`"  # absolutized and normalized
}
