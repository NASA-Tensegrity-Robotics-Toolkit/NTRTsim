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
 * @file AppRBStringConnections.cpp
 * @brief Contains the definition function main() for testing the rigid
 * body string connections with the 
 * Depricated as of version 1.1.0
 * @author Brian Tietz
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

// This application
#include "ContactTestModel.h"
#include "ContactTestController.h"
// This library
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
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
    std::cout << "AppContactTestModel" << std::endl;

    // First create the world
    const tgWorld::Config config(981); // gravity, cm/sec^2
    tgWorld world(config);

    // Second create the view
    const double timestep_physics = 1.0/2000.0;
    const double timestep_graphics = 1.f/60.f;
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);

    // Third create the simulation
    tgSimulation simulation(view);

    // Fourth create the models with their controllers and add the models to the
    // simulation
    ContactTestModel* const myModel = new  ContactTestModel();
    ContactTestController* const myController = new ContactTestController();
    myModel->attach(myController);
    
    simulation.addModel(myModel);
    
    simulation.run(1000);

    //Teardown is handled by delete, so that should be automatic
    return 0;
}
