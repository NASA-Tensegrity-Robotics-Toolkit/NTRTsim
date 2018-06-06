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

// This application
#include "T12Model.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgSimulation.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgWorld.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <iostream>
#include <string>
#include <vector>
// Sensors
#include "sensors/tgDataLogger2.h"
#include "sensors/tgRodSensorInfo.h"
#include "sensors/tgSpringCableActuatorSensorInfo.h"
// Controllers
#include "T12Controller.h"

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @param[in] argv argv[1] is the path of the YAML encoded structure
 * @return 0
 
*/
const bool  useGraphics = false;
const double initialLength = 1.0;
const double startTime = 1; // How long after the simulation the controller starts
const double timestep_physics = 0.0001; // seconds // from Hannah: recommended 0.0001, from earlier: recommended 0.001
const double timestep_graphics = 1.f/60.f; // seconds


void simulateNoGraphics() { 
    int nEpisodes =100; // Number of episodes ("trial runs")
    int nSteps = 600000; // Number of steps in each episode, 600k is 60 seconds (timestep_physics*nSteps)
    
    // Create the ground and world. Specify ground rotation in radians
    const double yaw = 0.0;
    const double pitch = 0.0;
    const double roll = 0.0;
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // the world will delete this
    tgBoxGround* ground = new tgBoxGround(groundConfig);

    const tgWorld::Config config(98.1); // gravity, dm/s^2
    tgWorld world(config, ground);

    tgSimView *view;
    // Create the view
    view = new tgSimView (world, timestep_physics, timestep_graphics) ;
    //view = new tgSimView (world);
    
    // Create the simulation
    tgSimulation simulation(*view);

    // Create the models with their controllers and add the models to the simulation
    T12Model* const myModel = new T12Model(); // second argument not necessary

    // Select controller to be used 
    T12Controller* const myController = new T12Controller(myModel, initialLength, startTime);

    // Attach the controller to the model 
    myModel->attach(myController);

    // Add the model to the world
    simulation.addModel(myModel);

    // Get file name (once) for saving all data to one file
    myController->getFileName();

    for (int i = 0; i<nEpisodes; i++) { 
	simulation.run(nSteps);
        myController->onTeardown(*myModel);
	simulation.reset();
    }
    // teardown is handled by delete
   // delete myModel;
   // delete view;
    //delete myController;
    //delete ground;
}

void simulateWithGraphics(void) {

    // Create the ground and world. Specify ground rotation in radians
    const double yaw = 0.0;
    const double pitch = 0.0;
    const double roll = 0.0;
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // the world will delete this
    tgBoxGround* ground = new tgBoxGround(groundConfig);

    const tgWorld::Config config(98.1); // gravity, dm/s^2
    tgWorld world(config, ground);

    tgSimView *view;
    // Create the view
    view = new tgSimViewGraphics (world, timestep_physics, timestep_graphics);

    // Create the simulation
    tgSimulation simulation(*view);

    // Create the models with their controllers and add the models to the simulation
    T12Model* const myModel = new T12Model(); // second argument not necessary

    // Select controller to be used 
    T12Controller* const myController = new T12Controller(myModel, initialLength, startTime);

    // Attach the controller to the model 
    myModel->attach(myController);

    // Add the model to the world
    simulation.addModel(myModel);

    // Run simulation
    simulation.run();
    
    // teardown is handled by delete
    delete myModel;
    delete view;
    delete myController;
    delete ground;
}

int main(int argc, char** argv)
{
    std::cout << "---------------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "App12BarCpp" << std::endl;
//    std::cout << "Graphics = " << useGraphics << std::endl;

    if(useGraphics) {
        simulateWithGraphics();
    } else { 
	simulateNoGraphics();
    }

    //return 0;


}
