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
 * @file AppSUPERball.cpp
 * @brief Contains the definition function main() for the Super Ball applicaiton
 * application.
 * $Id$
 */

// This application
#include "T6Model.h"
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

tgBoxGround *createGround();
tgWorld *createWorld();
tgSimViewGraphics *createGraphicsView(tgWorld *world);
tgSimView *createView(tgWorld *world);
void simulate(tgSimulation *simulation);

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */
int main(int argc, char** argv)
{
    std::cout << "AppSUPERball" << std::endl;

    // First create the ground and world
    tgWorld *world = createWorld();

	// Second create the view
	tgSimViewGraphics *view = createGraphicsView(world); // For visual experimenting on one tensegrity
//	tgSimView       *view = createView(world);         // For running multiple episodes

	// Third create the simulation
	tgSimulation *simulation = new tgSimulation(*view);

    // Fourth create the models with their controllers and add the models to the
    // simulation
    T6Model* const myModel = new T6Model();

    // Fifth, select the controller to use. Uncomment desired controller.


//    myModel->attach(pTC);
    simulation->addModel(myModel);
    
    // Run specified times within function call
    simulate(simulation);

    // Run until the user stops
//    simulation->run();

    //Teardown is handled by delete, so that should be automatic
    return 0;
}

tgBoxGround *createGround() {
    // Determine the angle of the ground in radians. All 0 is flat
    const double yaw = 0.0;
    const double pitch = 0.0;
    const double roll = 0.0;
    const btVector3 eulerAngles = btVector3(yaw, pitch, roll);  // Default: (0.0, 0.0, 0.0)
    const double friction = 0.5; // Default: 0.5
    const double restitution = 0.0;  // Default: 0.0
    const btVector3 size = btVector3(10000.0, 2, 10000.0); // Default: (500.0, 1.5, 500.0)
    const btVector3 origin = btVector3(0.0, 0.0, 0.0); // Default: (0.0, 0.0, 0.0)
    const tgBoxGround::Config groundConfig(eulerAngles, friction, restitution,
                                           size, origin);
    // the world will delete this
    return new tgBoxGround(groundConfig);
}

tgWorld *createWorld() {
    const tgWorld::Config config(98.1); // gravity, cm/sec^2  Use this to adjust length scale of world.
    // NB: by changing the setting below from 981 to 98.1, we've
    // scaled the world length scale to decimeters not cm.

    tgBoxGround* ground = createGround();
    return new tgWorld(config, ground);
}

/** Use for displaying tensegrities in simulation */
tgSimViewGraphics *createGraphicsView(tgWorld *world) {
    const double timestep_physics = 1.0 / 60.0 / 10.0; // Seconds
    const double timestep_graphics = 1.f /60.f; // Seconds, AKA render rate. Leave at 1/60 for real-time viewing
    return new tgSimViewGraphics(*world, timestep_physics, timestep_graphics);
}

/** Use for trial episodes of many tensegrities in an experiment */
tgSimView *createView(tgWorld *world) {
    const double timestep_physics = 1.0 / 60.0 / 10.0; // Seconds
    const double timestep_graphics = 1.f /60.f; // Seconds, AKA render rate. Leave at 1/60 for real-time viewing
    return new tgSimView(*world, timestep_physics, timestep_graphics);
}

/** Run a series of episodes for nSteps each */
void simulate(tgSimulation *simulation) {
    int nEpisodes = 1; // Number of episodes ("trial runs")
    int nSteps = 60000; // Number of steps in each episode, 60k is 100 seconds (timestep_physics*nSteps)
    for (int i=0; i<nEpisodes; i++) {
        simulation->run(nSteps);
        simulation->reset();
    }
}
