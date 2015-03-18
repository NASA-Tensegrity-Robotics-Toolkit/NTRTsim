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
* @file AppObstacleTest.cpp
* @brief Contains the definition function main() for testing new obstacles
* @author Brian Mirletz
* $Id$
*/
// This application
#include "tgBlockField.h"
#include "tgStairs.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/terrain/tgEmptyGround.h"
#include "core/terrain/tgHillyGround.h"
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
#include "tgcreator/tgUtil.h"

// The Bullet Physics Library
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
// The C++ Standard Library
#include <iostream>
/**
* The entry point.
* @param[in] argc the number of command-line arguments
* @param[in] argv argv[0] is the executable name
* @return 0
*/
int main(int argc, char** argv)
{

	std::cout << "AppObstacleTest" << std::endl;

	// First create the ground and world. Specify ground rotation in radians
	const double yaw = 0.0;
	const double pitch = 0.0;
	const double roll = 0.0;
	const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
	
	// the world will delete this
	tgBoxGround* ground = new tgBoxGround(groundConfig);

    btVector3 eulerAngles = btVector3(0.0, 0.0, 0.0);
   btScalar friction = 0.5;
   btScalar restitution = 0.0;
   // Size doesn't affect hilly terrain
   btVector3 size = btVector3(0.0, 0.1, 0.0);
   btVector3 origin = btVector3(0.0, 0.0, 0.0);
   size_t nx = 100;
   size_t ny = 100;
   double margin = 0.5;
   double triangleSize = 5.0;
   double waveHeight = 3.0;
   double offset = 0.0;
    tgHillyGround::Config hillGroundConfig(eulerAngles, friction, restitution,
                                    size, origin, nx, ny, margin, triangleSize,
                                    waveHeight, offset);
    

	const tgWorld::Config config(98.1); // gravity, cm/sec^2
	tgWorld world(config, ground);
	
	// Second create the view
	const double timestep_physics = 1.0/1000.0; // seconds
	const double timestep_graphics = 1.f/60.f; // seconds
	tgSimView view(world, timestep_physics, timestep_graphics);
	
	// Third create the simulation
	tgSimulation simulation(view);
    
    btVector3 position(0.0, 0.0, 0.0);

	// Fourth create the models with their controllers and add the models to the
	// simulation
    tgBlockField* myObstacle = new tgBlockField();
    // Add the model to the world
    simulation.addModel(myObstacle);
    
	tgStairs* bigStairs = new tgStairs();
	// Add the stairs to the world
	simulation.addObstacle(bigStairs);

    for (int i = 0; i < 3; i++)
    {
    
        simulation.run(10);
        
        if (i %2 == 0)
        {
            // World will delete prior pointer, so make a new one each time
            tgHillyGround* hillGround = new tgHillyGround(hillGroundConfig);
            simulation.reset(hillGround);
        }
        else
        {
            ground = new tgBoxGround(groundConfig);
            simulation.reset(ground);
        }        
    }
	return 0;
}
