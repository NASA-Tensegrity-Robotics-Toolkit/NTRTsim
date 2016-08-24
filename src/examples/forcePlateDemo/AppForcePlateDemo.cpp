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
 * @file AppForcePlateDemo.cpp
 * @brief Contains the definition function main() for the Force Plate Demo
 * application.
 * $Id$
 */

// This application
#include "sensors/forceplate/ForcePlateModel.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
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
    std::cout << "AppForcePlateDemo" << std::endl;

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
    const double timestep_physics = 0.001; // Seconds
    const double timestep_graphics = 1.f/60.f; // Seconds
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);

    // Third create the simulation
    tgSimulation simulation(view);

    // Fourth create the models with their controllers and add the models to the
    // simulation
    //T6Model* const myModel = new T6Model();

    // For this demo, create the force plate by passing in its config struct.
    // There is no need to create another model file here, since the
    // force plate is already a model.
    // 
    double L = 10.0;
    
    ForcePlateModel::Config forcePlateConfig(L);
    // This line determines the location of the force plate at (0,0,0).
    btVector3 forcePlateLocation = btVector3(0,3,0);
    // The force plate takes a boolean that turns debugging information on or off.
    // This is optional: the constructor defaults to "off"/"false".
    bool forcePlateDebugging = true;
    // Create the force plate model.
    tgModel* myModel = new ForcePlateModel(forcePlateConfig, forcePlateLocation,
					   forcePlateDebugging);

    // Fifth, select the controller to use, and attach it to the model.
    // For example, you could run the following to use the T6TensionController:
    //T6TensionController* const pTC = new T6TensionController(10000);
    //myModel->attach(pTC);

    // Finally, add out model to the simulation
    simulation.addModel(myModel);

    // We can also add in another model to the simulation.
    // This is how the force plate is used: the plate is added,
    // then the actual tensegrity system model is added.
    //@TO-DO pick a model.
    
    // Run until the user stops
    simulation.run();

    //Teardown is handled by delete, so that should be automatic
    return 0;
}
