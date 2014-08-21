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
 * @file AppEscape_T6.cpp
 * @brief Contains the definition function main() for the Escape T6
 * application.
 * $Id$
 */

// This application
#include "Escape_T6Model.h"
#include "Escape_T6Controller.h"

// This library
#include "core/terrain/tgHillyGround.h"
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"

// Bullet Physics
#include "LinearMath/btVector3.h"

// The C++ Standard Library
#include <iostream>

/**
 * Runs a series of nEpisodes episodes. 
 * Each episode tests a given control pattern for nSteps.
 * The fitness function (reward metric) for this experiment is 
 *     the maximum distance from the tensegrity's starting point 
 *     at any point during the episode
 * NB: Running episodes and using graphics are mutually exclusive features
 */
int main(int argc, char** argv)
{
    int nEpisodes = 10; // Number of episodes ("trial runs")
    int nSteps = 60000; // Number of steps in each episode

    std::cout << "AppEscape_T6" << std::endl;

    // First create the ground and world

    // Determine the angle of the ground in radians. All 0 is flat
    const double yaw = 0.0;
    const double pitch = 0.0;
    const double roll = 0.0;
    const tgHillyGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // the world will delete this
    tgHillyGround* ground = new tgHillyGround(groundConfig);

    const tgWorld::Config config(98.1); // gravity, cm/sec^2  Use this to adjust length scale of world.
    // Note, by changing the setting below from 981 to 98.1, we've
    // scaled the world length scale to decimeters not cm.
    tgWorld world(config, ground);

    // Second create the view
    const double timestep_physics = 1.0 / 60.0 / 10.0; // Seconds
    const double timestep_graphics = 1.f /60.f; // Seconds, AKA render rate

    //tgSimViewGraphics view(world, timestep_physics, timestep_graphics); // For display
    tgSimView view(world, timestep_physics, timestep_graphics); // For trial episodes

    // Third create the simulation
    tgSimulation simulation(view);

    // Fourth create the models with their controllers and add the models to the simulation
    Escape_T6Model* const myModel = new Escape_T6Model();

    // Fifth, select the controller to use. Uncomment desired controller.
    // Note for the above scale of gravity, this is in decimeters.

    Escape_T6Controller* const pTC = new Escape_T6Controller(9);

    myModel->attach(pTC);
    simulation.addModel(myModel);

    // Run a series of episodes for nSteps each
    for (int i=0; i<nEpisodes; i++)
    {   
        simulation.run(nSteps);
        simulation.reset();
    }

    //Teardown is handled by delete, so that should be automatic
    return 0;
}

