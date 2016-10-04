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
 * @file Appv3Ball.cpp
 * @brief Contains the definition function main() for the v3Ball 
 * application.
 * $Id$
 */

// This application
#include "v3Model.h"
#include "v4Model.h"
#include "v3TensionController.h"
#include "v4TensionController.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
#include "LinearMath/btScalar.h"
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
    //std::cout << "Appv3BallNew" << std::endl;

    // First create the ground and world
    
    // Determine the angle of the ground in radians. All 0 is flat
    const double yaw = 0.0;
    //const double pitch = M_PI/15.0;
    const double pitch = 0.0;
    const double roll = 0.0;
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // the world will delete this
    tgBoxGround* ground = new tgBoxGround(groundConfig);
    
    const tgWorld::Config config(98.1); // gravity, cm/sec^2  Use this to adjust length scale of world.
        // Note, by changing the setting below from 981 to 98.1, we've
        // scaled the world length scale to decimeters not cm.

    tgWorld world(config, ground);

    // Second create the view
    const double timestep_physics = 0.0001; // Seconds
    const double timestep_graphics = 0.01; // Seconds
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);

    // Third create the simulation
    tgSimulation simulation(view);

    // Fourth create the models with their controllers and add the models to the
    // simulation
    v3Model* const myModel = new v3Model();
    // v4Model* const myModel = new v4Model();

    // Fifth, select the controller to use, and attach it to the model.
    // For example, you could run the following to use the v3TensionController:
    btVector3 goalTrajectory = btVector3(0,0,0);
    //v3TensionController* const pTC = new v3TensionController(10000, timestep_physics, goalTrajectory);
    //v4TensionController* const pTC = new v4TensionController(10000, timestep_physics, goalTrajectory);
    //myModel->attach(pTC);
    //hisModel->attach(pTC);
    // Finally, add out model to the simulation
    //simulation.addModel(hisModel);
    simulation.addModel(myModel);
    
    
    // Run until the user stops
    simulation.run();

    //Teardown is handled by delete, so that should be automatic
    return 0;
}
