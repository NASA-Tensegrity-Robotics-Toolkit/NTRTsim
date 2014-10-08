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
* @file WorldConf_Spines_test.cpp
* @brief Contains a test suite for FileHelpers.
* $Id$
*/

// This application
#include "examples/learningSpines/TetrahedralComplex/FlemonsSpineModelLearning.h"
#include "examples/learningSpines/BaseSpineCPGControl.h"
// This library
#include "core/tgModel.h"
#include "core/tgSimView.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
// The C++ Standard Library
#include <iostream>
#include "gtest/gtest.h"
#include "helpers/FileHelpers.h"

using namespace std;

namespace {

	// The fixture for testing class FileHelpers.
	class SpinesTest : public ::testing::Test {
		protected:
			// You can remove any or all of the following functions if its body
			// is empty.
			
			SpinesTest() {
					
			}
			
			virtual ~SpinesTest() {
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

	TEST_F(SpinesTest, WorldConf_Spines) {
						// First create the world
				const tgWorld::Config config(981); // gravity, cm/sec^2
				tgWorld world(config); 

				// Second create the view
				const double stepSize = 1.0/1000.0; // Seconds
				const double renderRate = 1.0/60.0; // Seconds
				tgSimView view(world, stepSize, renderRate);

				// Third create the simulation
				tgSimulation simulation(view);

				// Fourth create the models with their controllers and add the models to the
				// simulation
				const int segments = 12;
				FlemonsSpineModelLearning* myModel =
				  new FlemonsSpineModelLearning(segments);

				/* Required for setting up learning file input/output. */
				const std::string suffix("default");
				
				const int segmentSpan = 3;
				const int numMuscles = 8;
				const int numParams = 2;
				const int segNumber = 6; // For learning results
				const double controlTime = .001;
				const double lowPhase = -1 * M_PI;
				const double highPhase = M_PI;
				const double lowAmplitude = -30.0;
				const double highAmplitude = 30.0;
				BaseSpineCPGControl::Config control_config(segmentSpan, numMuscles, numMuscles, numParams, segNumber, controlTime, 
															lowAmplitude, highAmplitude, lowPhase, highPhase);
				BaseSpineCPGControl* const myControl =
				  new BaseSpineCPGControl(control_config, suffix);
				myModel->attach(myControl);
				
				simulation.addModel(myModel);
		//EXPECT_EQ("test string\n", fileData);
	}

} // namespace

int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
