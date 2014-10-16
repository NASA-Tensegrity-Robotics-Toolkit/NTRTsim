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

helper_pwd=$(dirname "${BASH_SOURCE[0]}")
BASE_DIR="`( cd \"$helper_pwd/../../../\" && pwd )`"

BIN_DIR="${BASE_DIR}/bin"
CONF_DIR="${BASE_DIR}/conf"
ENV_DIR="${BASE_DIR}/env"
SRC_DIR="${BASE_DIR}/src"
TEST_DIR="${BASE_DIR}/test"
INTEGRATION_TEST_DIR="${BASE_DIR}/test_integration"
BUILD_DIR="${BASE_DIR}/build"
BUILD_TEST_DIR="${BASE_DIR}/build_test"
BUILD_INTEGRATION_TEST_DIR="${BASE_DIR}/build_test_integration"

SETUP_DIR="${BIN_DIR}/setup"
SHELL_UTILITIES_DIR="${BIN_DIR}/utilities"

DOWNLOADS_DIR="${ENV_DIR}/downloads"
ENV_BUILD_DIR="${ENV_DIR}/build"
ENV_BIN_DIR="${ENV_DIR}/bin"
LIB_DIR="${ENV_DIR}/lib"
INCLUDE_DIR="${ENV_DIR}/include"

ENV_NAME='env'
