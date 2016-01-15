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
* @file MuscleNP_test.cpp
* @brief Contains a test of conservation of momentum, energy, contact for MuscleNP
* $Id$
*/

// This application
#include "dev/btietz/muscleNPEnergy/MuscleNPCons.h"
// This library
#include "core/tgModel.h"
#include "core/tgSimView.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
#include "core/terrain/tgEmptyGround.h"

#include "LinearMath/btVector3.h"

// The C++ Standard Library
#include <iostream>
#include <fstream>
// Google Test
#include "gtest/gtest.h"


using namespace std;

namespace {

	// The fixture for testing class FileHelpers.
	class MuscleNPTest : public ::testing::Test {
		protected:
			// You can remove any or all of the following functions if its body
			// is empty.
			
			MuscleNPTest() {
					
			}
			
			virtual ~MuscleNPTest() {
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

	TEST_F(MuscleNPTest, MuscleNPMomentum) {

				// First create the world
				const tgWorld::Config config(0.0); // gravity, cm/sec^2
				tgEmptyGround* ground = new tgEmptyGround();
				tgWorld world(config, ground); 

				// Second create the view
				const double stepSize = 1.0/1000.0; // Seconds
				const double renderRate = 1.0/60.0; // Seconds
				tgSimView view(world, stepSize, renderRate);

				// Third create the simulation
				tgSimulation simulation(view);

				// Fourth create the models with their controllers and add the models to the
				// simulation
				MuscleNPCons* myModel = new MuscleNPCons();

				simulation.addModel(myModel);
				
				double energyStart = myModel->getEnergy();
				btVector3 momentumStart = myModel->getMomentum();
				btVector3 velocityStart = myModel->getVelocityOfBody(2);
				
				simulation.run(10000);
				
				double energyEnd = myModel->getEnergy();
				btVector3 momentumEnd = myModel->getMomentum();
				btVector3 velocityEnd = myModel->getVelocityOfBody(2);
				
                EXPECT_FLOAT_EQ(momentumStart[0], momentumEnd[0]);
                EXPECT_NEAR(momentumStart[1], momentumEnd[1], 1E-10); //TODO can this be pushed down?
				EXPECT_FLOAT_EQ(momentumStart[2], momentumEnd[2]);
                    
                EXPECT_LE(energyEnd, energyStart);
				
	}

} // namespace

int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
