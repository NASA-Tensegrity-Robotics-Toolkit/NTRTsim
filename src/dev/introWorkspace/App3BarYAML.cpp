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
 * @file AppHorizontalSpine.cpp
 * @brief Contains the definition function main() for App3BarYAML
 * which builds an example 3 bar prism using YAML.
 * @author Andrew Sabelhaus
 * $Id$
 */

// This application
#include "yamlbuilder/TensegrityModel.h"
#include "LengthControllerYAML.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgSimulation.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimView.h"
#include "core/tgWorld.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <iostream>
#include <string>
#include <vector>
#include "sensors/tgDataLogger2.h"
#include "sensors/tgRodSensorInfo.h"
#include "sensors/tgSpringCableActuatorSensorInfo.h"

#define USEGRAPHICS 0
#define LOGDATA 0

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @param[in] argv argv[1] is the path of the YAML encoded structure
 * @return 0
 */
int main(int argc, char** argv)
{
    // For this YAML parser app, need to check that an argument path was
    // passed in.
    if (argv[1] == NULL)
    {
      throw std::invalid_argument("No arguments passed in to the application. You need to specify which YAML file you wouldd like to build.");
    }
 
    // create the ground and world. Specify ground rotation in radians
    const double yaw = 0.0;
    const double pitch = 0.0;
    const double roll = 0.0;
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // the world will delete this
    tgBoxGround* ground = new tgBoxGround(groundConfig);

    const tgWorld::Config config(98.1); // gravity, dm/sec^2
    tgWorld world(config, ground);

    // create the view
    //const double timestep_physics = 0.0001; // seconds
    const double timestep_physics = 0.001;
    const double timestep_graphics = 1.f/60.f; // seconds
    #if(USEGRAPHICS)
        tgSimViewGraphics view(world, timestep_physics, timestep_graphics);
    #else
        tgSimView view(world, timestep_physics, timestep_graphics);
    #endif

    // create the simulation
    tgSimulation simulation(view);

    // create the models with their controllers and add the models to the simulation
    // This constructor for TensegrityModel takes the 'debugging' flag as the
    // second argument.
    TensegrityModel* const myModel = new TensegrityModel(argv[1],false);

    // Attach a controller to the model, if desired.
    // This is a controller that interacts with a generic TensegrityModel as
    // built by the TensegrityModel file.

    // Parameters for the LengthControllerYAML are specified in that .h file,
    // repeated here:
    double startTime = 5.0;
    double minLength = 0.7;
    double rate = 1.5; //0.25
    std::vector<std::string> tagsToControl;
    // See the threeBarModel.YAML file to see where "vertical_string" is used.
    tagsToControl.push_back("horizontal_string");
    
    // Create the controller
    // FILL IN 6.6 HERE
    LengthControllerYAML* const myController = new LengthControllerYAML(startTime, minLength, rate, tagsToControl);
    
    // Attach the controller to the model
    // FILL IN 6.7 HERE
    myModel->attach(myController);

    // Add the model to the world
    simulation.addModel(myModel);
    
    #if(LOGDATA)
        // Add sensors using the new sensing framework
        // A string prefix for the filename
        std::string log_filename = "~/projects/tg_shared/App3BarYAML";
        // The time interval between sensor readings:
        double timeInterval = 0.2;
        // First, create the data manager
        tgDataLogger2* myDataLogger = new tgDataLogger2(log_filename,timeInterval);
        //std::cout << myDataLogger->toString() << std::endl;
        // Then, add the model to the data logger
        myDataLogger->addSenseable(myModel);
        // Create sensor infos for all the types of sensors that the data logger
        // will create.
        tgRodSensorInfo* myRodSensorInfo = new tgRodSensorInfo();
        tgSpringCableActuatorSensorInfo* mySCASensorInfo =
          new tgSpringCableActuatorSensorInfo();
        // Attach the sensor infos to the data logger
        myDataLogger->addSensorInfo(myRodSensorInfo);
        myDataLogger->addSensorInfo(mySCASensorInfo);
        // Next, attach it to the simulation
        simulation.addDataManager(myDataLogger);
    #endif


    #if(USEGRAPHICS)
        simulation.run();
    
    #else
    {
        int nEpisodes = 100;  // Number of episodes ("trial runs")
        int nSteps = 10001; // Number of steps in each episode, 60k is 60 seconds (timestep_physics*nSteps)
        for (int i=1; i<=nEpisodes; i++)
        {
            std::cout << "Running episode " << i << " of " << nEpisodes << std::endl;
            if(i!=1)
            {
                std::cout << "RESET" << std::endl;
                simulation.reset();
                myController->resetTimePassed();
            }
            simulation.run(nSteps);
        }
    }    
    #endif

    // teardown is handled by delete
    return 0;
}
