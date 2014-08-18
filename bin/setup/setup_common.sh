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

# Purpose: Common setup code
# Date:    2014-08-18

###############################
# Configuration
local_setup_path="`dirname \"$0\"`"                # relative
base_dir="`( cd \"$local_setup_path/../../\" && pwd )`"  # absolutized and normalized
install_conf_file="$base_dir/conf/install.conf"
if [ ! -f "$install_conf_file" ]; then
	echo "Missing install.conf ($install_conf_file). Please fix this and try again."
	exit 1
fi
source "$install_conf_file"
###############################


