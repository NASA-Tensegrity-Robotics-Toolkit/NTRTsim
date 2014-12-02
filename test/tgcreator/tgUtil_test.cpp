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

/**
* @file tgUtil_test.cpp
* @brief Contains a test of the vector and quaternion manipulations in
* tgUtil
* $Id$
*/

// This application
#include "tgcreator/tgUtil.h"
// The Bullet Physics Library
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btMatrix3x3.h"
// The C++ Standard Library
#include <iostream>
#include <fstream>
// Google Test
#include "gtest/gtest.h"


using namespace std;

namespace {

	// The fixture for testing class FileHelpers.
	class tgUtilTest : public ::testing::Test {
		protected:
			// You can remove any or all of the following functions if its body
			// is empty.
			
			tgUtilTest() {
					
			}
			
			virtual ~tgUtilTest() {
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

	TEST_F(tgUtilTest, testQuaternion) {
				
				// This is a key test since we define btTransforms against the up axis
				btVector3 up(0.0, 1.0, 0.0);
				btVector3 down(0.0, -1.0, 0.0);
				
				btQuaternion testQuaternion = tgUtil::getQuaternionBetween(down, up);
				
				btVector3 result = down.rotate(testQuaternion.getAxis(), testQuaternion.getAngle());				
				
				// Just comparing the vectors results in a floating point residual, which causes the test to fail
				EXPECT_TRUE((up - result).fuzzyZero());
				
				// Test another arbitrary opposite vector
				btVector3 start(1.0, -1.0, 2.0);
				btVector3 end (-1.0, 1.0, -2.0);
				
				testQuaternion = tgUtil::getQuaternionBetween(start, end);
				
				result = end.rotate(testQuaternion.getAxis(), testQuaternion.getAngle());	

				EXPECT_TRUE((start - result).fuzzyZero());
	}

} // namespace

int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
