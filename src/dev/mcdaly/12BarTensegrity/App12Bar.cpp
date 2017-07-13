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
#include "yamlbuilder/TensegrityModel.h"
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
#include "LengthController12BarCube.h"

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

    // Create the ground and world. Specify ground rotation in radians
    const double yaw = 0.0;
    const double pitch = 0.0;
    const double roll = 0.0;
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // the world will delete this
    tgBoxGround* ground = new tgBoxGround(groundConfig);

    const tgWorld::Config config(98.1); // gravity, dm/s^2
    tgWorld world(config, ground);

    // Create the view
    const double timestep_physics = 0.0001; // seconds // recommened 0.001
    const double timestep_graphics = 1.f/60.f; // seconds
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);

    // Create the simulation
    tgSimulation simulation(view);

    // Create the models with their controllers and add the models to the simulation
    TensegrityModel* const myModel = new TensegrityModel(argv[1],false); // second argument not necessary

    // Parameters for the LengthController12BarCube are specified in that .h file,
    // repeated here:
    double startTime = 5;
    double minLength = 0.1;
    double rate = 1;
    bool loop = true;
    std::vector<std::string> tagsToControl;
    tagsToControl.push_back("actuated_cable");
    
    // Create the controller
    LengthController12BarCube* const myController = new LengthController12BarCube(startTime, minLength, rate, loop, tagsToControl);
    
    // Attach the controller to the model
    myModel->attach(myController);

    // Create data logger
    std::string log = "~/12-bar-tensegrity/NTRT_logs/log";
    tgDataLogger2* myDataLogger = new tgDataLogger2(log);
    myDataLogger->addSenseable(myModel);

    // Create two sensor infos, one for tgRods and other for tgSpringCableActuators
    tgRodSensorInfo* myRodSensorInfo = new tgRodSensorInfo();
    tgSpringCableActuatorSensorInfo* mySCASensorInfo = new tgSpringCableActuatorSensorInfo();
    myDataLogger->addSensorInfo(myRodSensorInfo);
    myDataLogger->addSensorInfo(mySCASensorInfo);
    
    // Add data logger to the world
    //simulation.addDataManager(myDataLogger); // comment/uncomment to record data

    // Add the model to the world
    simulation.addModel(myModel);

    // Run simulation
    simulation.run();

    // teardown is handled by delete
    return 0;
}
