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
 * @file AppT6Model_tgDLR.cpp
 * @brief Contains the definition function main() for the redesign example of tgDL
 * application.
 * $Id$
 */

// This application
#include "T6Model_tgDLR.h"
#include "T6RestLengthController_tgDLR.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
#include "tgDataObserver_tgDLR.h"
#include "tgDataLoggerRodBasic.h"
#include "tgDataLoggerRodFullState.h"
#include "tgDataLoggerLinearStringBasic.h"
#include "tgDataLogger_tgDLR.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <iostream>
#include <string>

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */
int main(int argc, char** argv)
{
    std::cout << "AppT6Model_tgDLR" << std::endl;

    // First create the ground and world
    
    // Determine the angle of the ground in radians. All 0 is flat
    const double yaw = 0.0;
    const double pitch = M_PI/15.0;
    const double roll = 0.0;
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // the world will delete this
    tgBoxGround* ground = new tgBoxGround(groundConfig);

    // Gravity is 98.1, so our length unit is decimeters
    const tgWorld::Config config(98.1);
    tgWorld world(config, ground);

    // Second create the view
    const double timestep_physics = 0.0001; // Seconds
    const double timestep_graphics = 1.f/60.f; // Seconds
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);

    // Third create the simulation
    tgSimulation simulation(view);

    // Fourth create the models with their controllers and add the models to the
    // simulation
    T6Model_tgDLR* const myModel = new T6Model_tgDLR();

    // Fifth, select the controller to use. Uncomment desired controller.

    // For the T6RestLengthController, pass in the amount of cable to contract
    // in. This is the "rest length difference": the static offset of cable
    // length between geometric length in equilibrium and the actual rest length
    // of an individual cable. 
    // Note for the above scale of gravity, this is in decimeters.
    T6RestLengthController_tgDLR* const pTC = new T6RestLengthController_tgDLR(4);

    // Note here that we need to be explicit about the namespace of attach(),
    // since the template needs to be for this subclass T6Model_tgDLR of tgModel.
    // TODO: make this properly polymorphic so we don't need to worry about
    // namespace issues
    myModel->tgSubject<T6Model_tgDLR>::attach(pTC);

    // Finally, attach a data observer (with some loggers) if we want to log data.
    // The filename prefix of the log files:
    std::string logFileNamePrefix = "/home/drew/NTRTsim_logs/tgDLR";

    // Create a vector of all the loggers we want to attach...
    std::vector<tgDataLogger_tgDLR*> loggers;

    //tgDataLogger_tgDLR* rodLogger = new tgDataLoggerRodBasic();
    tgDataLogger_tgDLR* rodLogger = new tgDataLoggerRodFullState();
    tgDataLogger_tgDLR* linearStringLogger = new tgDataLoggerLinearStringBasic();

    // add the loggers to the vector
    loggers.push_back(rodLogger);
    loggers.push_back(linearStringLogger);

    // Create the observer, passing in the dataloggers vector
    tgDataObserver_tgDLR* const myDataObserver = 
      new tgDataObserver_tgDLR(logFileNamePrefix, loggers);

    // Attach the observer
    myModel->attachtgModel(myDataObserver);

    // Add model to simulation...
    simulation.addModel(myModel);
    
    // And run until the user stops
    simulation.run();

    //Teardown is handled by delete, so that should be automatic
    // TODO: check if this is true for the data Loggers and Observer now, too!!
    return 0;
}
