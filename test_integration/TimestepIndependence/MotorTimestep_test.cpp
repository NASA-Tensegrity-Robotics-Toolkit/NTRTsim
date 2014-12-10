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
* either express or implied. See the License 197.632for the specific language
* governing permissions and limitations under the License.
*/

/**
* @file MotorTimestep_test.cpp
* @brief Contains a test ensuring our motor models are independent of
* the simulation timestep
* @author Brian Mirletz
* @date December 2014
* @version 1.1.0
* $Id$
*/

// This application
#include "dev/btietz/timestepTest/tsTestRig.h"
// This library
#include "core/tgSpringCableActuator.h"
#include "core/tgModel.h"
#include "core/tgSimView.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
#include "helpers/FileHelpers.h"
// The C++ Standard Library
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
// Google Test
#include "gtest/gtest.h"


using namespace std;

namespace {

	// The fixture for testing class FileHelpers.
	class MotorTest : public ::testing::Test {
		protected:
			// You can remove any or all of the following functions if its body
			// is empty.
			
			MotorTest() {
					
			}
			
			virtual ~MotorTest() {
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

	TEST_F(MotorTest, KinematicMotor) {
				// First create the world
				const tgWorld::Config config(981); // gravity, dm/sec^2
				tgWorld world(config); 

				// Second create the view
				const double stepSize = 1.0/1000.0; // Seconds
				const double renderRate = 1.0/60.0; // Seconds
				tgSimView view(world, stepSize, renderRate);

				// Third create the simulation
				tgSimulation simulation(view);

				// Fourth create the models with their controllers and add the models to the
				// simulation
				bool useKinematic = true;
				tsTestRig* const myModel = new tsTestRig(useKinematic);
				
				//tsTestRig automatically attempts to go to length of 5
				/// @todo consider adding a controller, or at least changing
				/// this setpoint based on an input
				
				simulation.addModel(myModel);
				
				simulation.run(1000);
				
				const std::vector<tgSpringCableActuator*>& testMuscles = myModel->getAllMuscles();
				
				// If this fails we've changed the model in unexpected ways
				ASSERT_EQ(testMuscles.size(), 1);
				
				double finalLength = testMuscles[0]->getRestLength();
				double finalTime = myModel->getTotalTime();
				
				simulation.reset();
				view.setStepSize(1.0/500.0);
				simulation.run(500);
				
				const std::vector<tgSpringCableActuator*>& newTestMuscles = myModel->getAllMuscles();
				
				// If this fails we've changed the model in unexpected ways
				ASSERT_EQ(newTestMuscles.size(), 1);
				EXPECT_FLOAT_EQ(finalTime, myModel->getTotalTime());
				
				double newLength = newTestMuscles[0]->getRestLength();
				
				std::cout << "Original Restlength " << finalLength << " New restlength: " << newLength << std::endl;
				
				EXPECT_NEAR(finalLength, newLength, 0.03); 
				
				simulation.reset();
				view.setStepSize(1.0/5000.0);
				simulation.run(5000);
				
				const std::vector<tgSpringCableActuator*>& thirdTestMuscles = myModel->getAllMuscles();
				
				// If this fails we've changed the model in unexpected ways
				ASSERT_EQ(thirdTestMuscles.size(), 1);
				EXPECT_FLOAT_EQ(finalTime, myModel->getTotalTime());
				
				double thirdLength = thirdTestMuscles[0]->getRestLength();
				
				std::cout << "Original Restlength " << finalLength << " New restlength: " << thirdLength << std::endl;
				
				// This value allows for multiple scales (error slightly higher with higher gravity
				EXPECT_NEAR(thirdLength, finalLength, 0.03); 
	}

	TEST_F(MotorTest, LinearMotor) {
				// First create the world
				const tgWorld::Config config(981); // gravity, dm/sec^2
				tgWorld world(config); 

				// Second create the view
				const double stepSize = 1.0/1000.0; // Seconds
				const double renderRate = 1.0/60.0; // Seconds
				tgSimView view(world, stepSize, renderRate);

				// Third create the simulation
				tgSimulation simulation(view);

				// Fourth create the models with their controllers and add the models to the
				// simulation
				bool useKinematic = false;
				tsTestRig* const myModel = new tsTestRig(useKinematic);
				
				//tsTestRig automatically attempts to go to length of 5
				/// @todo consider adding a controller, or at least changing
				/// this setpoint based on an input
				
				simulation.addModel(myModel);
				
				simulation.run(1000);
				
				const std::vector<tgSpringCableActuator*>& testMuscles = myModel->getAllMuscles();
				
				// If this fails we've changed the model in unexpected ways
				ASSERT_EQ(testMuscles.size(), 1);
				
				double finalLength = testMuscles[0]->getRestLength();
				double finalTime = myModel->getTotalTime();
				
				simulation.reset();
				view.setStepSize(1.0/500.0);
				simulation.run(500);
				
				const std::vector<tgSpringCableActuator*>& newTestMuscles = myModel->getAllMuscles();
				
				// If this fails we've changed the model in unexpected ways
				ASSERT_EQ(newTestMuscles.size(), 1);
				EXPECT_FLOAT_EQ(finalTime, myModel->getTotalTime());
				
				double newLength = newTestMuscles[0]->getRestLength();
				
				std::cout << "Original Restlength " << finalLength << " New restlength: " << newLength << std::endl;
				
				EXPECT_NEAR(finalLength, newLength, 0.03); 
				
				simulation.reset();
				view.setStepSize(1.0/5000.0);
				simulation.run(5000);
				
				const std::vector<tgSpringCableActuator*>& thirdTestMuscles = myModel->getAllMuscles();
				
				// If this fails we've changed the model in unexpected ways
				ASSERT_EQ(thirdTestMuscles.size(), 1);
				EXPECT_FLOAT_EQ(finalTime, myModel->getTotalTime());
				
				double thirdLength = thirdTestMuscles[0]->getRestLength();
				
				std::cout << "Original Restlength " << finalLength << " New restlength: " << thirdLength << std::endl;
				
				/// @todo Fix acceleation problem in tgLinear string, tighten the bounds on this to rounding error
				EXPECT_NEAR(thirdLength, finalLength, 0.03); 
	}

} // namespace

int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
