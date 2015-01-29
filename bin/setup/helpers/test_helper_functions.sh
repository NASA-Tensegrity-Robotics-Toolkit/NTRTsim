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

source "helper_functions.sh"

function test_read_options()
{
    message="Do you want to test read_options?"
    options=("Y" "n" "a")
    default="Y"
    output=$(read_options "$message" $options $default)
    echo "read_options() returned '$output'"
}

function test_read_text()
{
    message="Do you want to test read_text?"
    default="I'd rather test the default."
    output=$(read_text "$message" "$default")
    echo "read_text() returned '$output'"
}

function test_relative_path()
{
    a="/this/is/an/absolute/path"
    b="/this/is/the/target/path"
    rel=$(get_relative_path "$a" "$b")
    echo "relative path is $rel (should be '../../../the/target/path')"
}

function test_create_directory_if_noexist()
{
    create_directory_if_noexist "/home/perry/meow/mix/dog/yow"
    create_directory_if_noexist "this/is/a/relative/dir"
}

function test_error_exit_if_create_directory_fail()
{
    create_directory_if_noexist "/cantwritehere/"
}

function test_has_command()
{
    if has_command $1; then
        echo "User has $1 installed."
    else
        echo "User does NOT have $1 installed."
    fi
}

function test_get_valid_link()
{
    download_file "http://www.perryb.ca/" "/home/perry/sandbox/works.html"
    download_file "http://www.perryb.ca/test.txt" "/home/perry/sandbox/404.txt"
    download_file "http://google.com:81/test.txt" "/home/perry/sandbox/timeout.txt"
}

function test_create_exist_symlink()
{
    create_exist_symlink "helper_paths.sh" "works"
    create_exist_symlink "helper_pathsinvalid.sh" "doesnotwork"
}
