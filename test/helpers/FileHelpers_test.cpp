/*
* Copyright © 2012, United States Government, as represented by the
* Administrator of the National Aeronautics and Space Administration.
* All rights reserved.
*
* The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
* under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
* http://www.apache.org/licenses/LICENSE-2.0.
*
* Unless required by applicable law or agreed to in writing,
* software distributed under the License is distributed on an
* "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
* either express or implied. See the License for the specific language
* governing permissions and limitations under the License.
*/

#include <iostream>
#include "gtest/gtest.h"
#include "helpers/FileHelpers.h"

using namespace std;

namespace {

    // The fixture for testing class FileHelpers.
    class FileHelpersTest : public ::testing::Test {
        protected:
            // You can remove any or all of the following functions if its body
            // is empty.

            FileHelpersTest() {
                // You can do set-up work for each test here.
            }

            virtual ~FileHelpersTest() {
                // You can do clean-up work that doesn't throw exceptions here.
            }

            // If the constructor and destructor are not enough for setting up
            // and cleaning up each test, you can define the following methods:

            virtual void SetUp() {
                // Code here will be called immediately after the constructor (right
                // before each test).
            }

            virtual void TearDown() {
                // Code here will be called immediately after each test (right
                // before the destructor).
            }

            // Objects declared here can be used by all tests in the test case for FileHelpers.
    };

    TEST_F(FileHelpersTest, ReadsFilesIntoString) {
        string fileData = FileHelpers::getFileString("test_file.txt");
        EXPECT_EQ("test string\n", fileData);
    }

}  // namespace

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
