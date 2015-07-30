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
 * @file AppTetraSpineCol.cpp
 * @brief Contains the definition function main() for the Tetra Spine Static
 * application.
 * @author Brian Mirletz
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

// This application
#include "TetraSpineCollisions.h"
#include "colSpineSine.h"
#include "Wall.h"
// This library
#include "core/tgModel.h"
#include "core/tgSimView.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
#include "core/terrain/tgHillyGround.h"
#include "examples/learningSpines/tgCPGLogger.h"
// obstacles
#include "models/obstacles/tgBlockField.h"
// The C++ Standard Library
#include <iostream>

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name; argv[1], if supplied, is the
 * suffix for the controller
 * @return 0
 */
int main(int argc, char** argv)
{
    std::cout << "AppTetraSpineHT" << std::endl;

    // First create the world
    const double scale = 100;
    const tgWorld::Config config(9.81 * scale); // gravity, cm/sec^2

	;
#if (1)
	btVector3 eulerAngles = btVector3(M_PI/4.0, 0.0, 0.0);
   btScalar friction = 0.5;
   btScalar restitution = 0.1;
   btVector3 size = btVector3(500.0, 1.5, 500.0);
   btVector3 origin = btVector3(0.0, 0.0, 0.0);
    const size_t nx = 100;
    const size_t ny = 100;
    const double triangleSize = 15.0;
    const double waveHeight = 5.0;
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
    tgSimViewGraphics view(world, stepSize, renderRate);

    // Third create the simulation
    tgSimulation simulation(view);

    // Fourth create the models with their controllers and add the models to the
    // simulation
    const int segments = 12;
    TetraSpineCollisions* myModel =
      new TetraSpineCollisions(segments, scale /2.0);
    
    colSpineSine* const myControl =
      new colSpineSine("controlVars.json", "tetraTerrain/");

    myModel->attach(myControl);
    /*
    tgCPGLogger* const myLogger = 
      new tgCPGLogger("logs/CPGValues.txt");
    
    myControl->attach(myLogger);
    */
	
	// Add obstacles
	btVector3 wallOrigin(0.0, 0.0, 50.0);
	Wall* myWall = new Wall(wallOrigin);
    
    tgBlockField* myObstacle = new tgBlockField();
    
    simulation.addModel(myModel);
    //simulation.addModel(myObstacle);
    
    int i = 0;
    while (i < 1)
    {
        simulation.run(240000);
        //simulation.reset();
        i++;
    }
    
    /// @todo Does the model assume ownership of the controller?
    /** No - a single controller could be attached to multiple subjects
    * However, having this here causes a segfault, since there is a call
    * to onTeardown() when the simulation is deleted
    */
    #if (0)
    delete myControl;
    #endif
    //Teardown is handled by delete, so that should be automatic
    return 0;
}
