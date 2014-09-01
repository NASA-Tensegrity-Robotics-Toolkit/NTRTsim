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

ENV_SRC_DIR="/home/bbadmin/env_BOOSTBULLET"
BUILDBOT_BUILD_DIR="/home/bbadmin/buildbot/slave/master/build"

if [ ! -d $ENV_SRC_DIR ]; then
    echo "Could not find environment source for Bullet and Boost. Searched at ${ENV_SRC_DIR}"
fi

if [ ! -d $BUILDBOT_BUILD_DIR ]; then
    echo "Could not find slave build directory. Searched at ${BUILDBOT_BUILD_DIR}"
fi

cp -r $ENV_SRC_DIR/env $BUILDBOT_BUILD_DIR || { echo "Encountered failure while copying Boost and Bullet environment pre-compile to BuildBot build directory."; exit 1; }
