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
 * @file AppStandingHexagonalSpineTestModel.cpp
 * @brief Contains the definition function main() for the StandingHexagonalSpineTestModel
 * application.
 * @author Lauren Sharo
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

// This application
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
// The C++ Standard Library
#include <iostream>

#include "StandingHexagonalSpineTestModel.h"
#include "models/obstacles/tgBlockField.cpp"


int main(int argc, char** argv)
{
	std::cout << "AppStandingHexagonalSpineTestModel" << std::endl;

	// First create the world
	const tgWorld::Config config(98.1); // gravity, cm/sec^2
	tgWorld world(config);

	// Second create the view
	const double stepSize = 1.0 / 2000.0; //Seconds
	tgSimViewGraphics view(world, stepSize);

	// Third create the simulation
	tgSimulation *simulation = new tgSimulation(view);

	// Fourth create the models and add them to the simulation
	const int topSegments = 1;
	const int bottomSegments = 1;
	StandingHexagonalSpineTestModel* const myModel =
			new StandingHexagonalSpineTestModel(topSegments, bottomSegments);
	simulation->addModel(myModel);

	// Block field area of effect
	btVector3 minPos = btVector3(-50.0, 0.0, -50.0);
	btVector3 maxPos = btVector3(50.0, 0.0, 50.0);
	btVector3 origin = btVector3(0, 0, 0);

	// Block Field obstacle:
	tgBlockField::Config blockConfigA(origin, 0.5, 0.0, minPos, maxPos, 750,
			1.0, 1.0, 4.5); // Small blocks
	tgBlockField::Config blockConfigB(origin, 0.5, 0.0, minPos, maxPos, 600,
			2.5, 2.5, 4.5); // Medium blocks
	tgBlockField::Config blockConfigC(origin, 0.5, 0.0, minPos, maxPos, 350,
			4.0, 4.0); // Large blocks
	tgBlockField* blocks = new tgBlockField(blockConfigC);

	//simulation->addModel(blocks);

	// Retrieves and prints the mass of the structure to the command line
	myModel->getMass();

	// Run until the user stops
	simulation->run();

	//Teardown is handled by delete, so that should be automatic
	return 0;
}
