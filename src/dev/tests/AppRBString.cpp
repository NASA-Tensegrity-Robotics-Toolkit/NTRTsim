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
 * @file AppFlemonsSpine.cpp
 * @brief Contains the definition function main() for the Flemons Spine
 * application.
 * @author Brian Tietz
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

// This application
#include "RBStringTest.h"
// This library
#include "core/tgModel.h"
#include "tgcreator/tgNode.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
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
    std::cout << "AppNestedStructureTest" << std::endl;

    // First create the world
    const tgWorld::Config config( 9.81); // gravity, cm/sec^2

    tgWorld world(config);

    // Second create the view
    const double stepSize = 1.0/1000.0; //Seconds   
    tgSimViewGraphics view(world, stepSize);

    // Third create the simulation
    tgSimulation simulation(view);

    // Fourth create the models with their controllers and add the models to the
    // simulation
    const int segments = 10;
    tgNode start(-10, 0, 0);
    tgNode end (5, 5.0, .1);
    
    
    const double radius  = 0.2;
    const double density = 2;  // Note: This needs to be high enough or things fly apart...
    const tgRod::Config rodConfig(radius, density);
    
    const double stiffness = 1000.0;
    const double damping = .01*stiffness;
    const tgBasicActuator::Config stringConfig(stiffness, damping);
    const RBStringTest::Config rbConfig(segments, rodConfig, stringConfig, 3.0);
    RBStringTest* myModel = new RBStringTest(&start, &end, rbConfig);
#if (0)
    NestedStructureSineWaves* muscleControl = new NestedStructureSineWaves();
    myModel->attach(*muscleControl);
#endif        
    simulation.addModel(myModel);
    
    simulation.run(1000);

    //Teardown is handled by delete, so that should be automatic
    return 0;
}
