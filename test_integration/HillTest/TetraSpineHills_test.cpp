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
* @file TetraSpineHills_test.cpp
* @brief Testing the reaction of the contact cables to terrain in a controller situation
* $Id$
*/

// This application
#include "dev/btietz/tetraCollisions/TetraSpineCollisions.h"
#include "dev/btietz/tetraCollisions/colSpineSine.h"
// This library
#include "core/terrain/tgHillyGround.h"
#include "core/tgModel.h"
#include "core/tgSimView.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
#include "helpers/FileHelpers.h"

// Bullet Physics
#include "LinearMath/btVector3.h"
// JSON
#include <json/json.h>
// The C++ Standard Library
#include <iostream>
#include <fstream>
// Google Test
#include "gtest/gtest.h"


using namespace std;

namespace {

	// The fixture for testing class FileHelpers.
	class HillsTest : public ::testing::Test {
		protected:
			// You can remove any or all of the following functions if its body
			// is empty.
			
			HillsTest () {
					
			}
			
			virtual ~HillsTest () {
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

	TEST_F(HillsTest, TetraSpine) {
                // First create the world
            
                const double scale = 100;
                const tgWorld::Config config(9.81 * scale); // gravity, cm/sec^2
#if (1)
                btVector3 eulerAngles = btVector3(M_PI/4.0, 0.0, 0.0);
                btScalar friction = 0.5;
                btScalar restitution = 0.1;
                btVector3 size = btVector3(500.0, 1.5, 500.0);
                btVector3 origin = btVector3(0.0, 0.0, 0.0);
                const size_t nx = 100;
                const size_t ny = 100;
                const double triangleSize = 2.0;
                const double waveHeight = 2.0;
                const double offset = 0.0;
                const double margin = 1.0;
                tgHillyGround::Config groundConfig(eulerAngles, friction, restitution,
                                                size, origin, nx, ny, margin, triangleSize,
                                                waveHeight, offset);
                
                tgHillyGround* ground = new tgHillyGround(groundConfig);
                
                tgWorld world(config, ground); 
#else
                tgWorld world(config); 
#endif
                
				// Second create the view
				const double stepSize = 1.0/1000.0; // Seconds
				const double renderRate = 1.0/60.0; // Seconds
				tgSimView view(world, stepSize, renderRate);

				// Third create the simulation
				tgSimulation simulation(view);

				// Fourth create the models with their controllers and add the models to the
				// simulation
				const int segments = 6;
				TetraSpineCollisions* myModel =
                    new TetraSpineCollisions(segments, scale /2.0);

				// Required for setting up learning file input/output.
				const std::string suffix("default");
				
				colSpineSine* const myControl =
                    new colSpineSine("controlVars.json", "tetraTerrain/");

                    myModel->attach(myControl);
				
				simulation.addModel(myModel);
				
				simulation.run(15000);
				simulation.reset();
				
                std::string controlFilePath = FileHelpers::getResourcePath("tetraTerrain/");
                std::string controlFile = controlFilePath + "controlVars.json";
				
                    Json::Value root; // will contains the root value after parsing.
                    Json::Reader reader;

                    bool parsingSuccessful = reader.parse( FileHelpers::getFileString(controlFile.c_str()), root );
                    if ( !parsingSuccessful )
                    {
                        // report to the user the failure and their locations in the document.
                        std::cout << "Failed scores at top level!\n"
                            << reader.getFormattedErrorMessages();
                        throw std::invalid_argument("Bad filename for JSON");
                    }
    
                double dist = root.get("scores", 0.0).asDouble();
    
				EXPECT_FLOAT_EQ(dist, 5.19875);
				
				// Will print out another set of dist moved on teardown
	}

} // namespace

int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
