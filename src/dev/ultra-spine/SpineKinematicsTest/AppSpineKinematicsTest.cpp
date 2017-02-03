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
 * @file AppSpineKinematicsTest.cpp
 * @brief Contains the definition function main() for AppSpineKinematicsTest
 * which tests out some kinematic movements of a horizontal spine
 * @author Drew Sabelhaus
 * $Id$
 */

// This application
#include "yamlbuilder/TensegrityModel.h"
#include "SpineKinematicsTestController.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgSimulation.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgWorld.h"
#include "sensors/tgDataLogger2.h"
#include "sensors/tgRodSensorInfo.h"
#include "sensors/tgSpringCableActuatorSensorInfo.h"
#include "sensors/tgCompoundRigidSensorInfo.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <iostream>
#include <string>
#include <vector>

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

    // For this kinematics test, no gravity. We want a purely kinematic movement,
    // as if the spine was floating in space.
    const tgWorld::Config config(0.0); // gravity, dm/sec^2
    tgWorld world(config, ground);

    // create the view
    const double timestep_physics = 0.0001; // seconds
    //const double timestep_physics = 0.001;
    const double timestep_graphics = 1.f/60.f; // seconds
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);

    // create the simulation
    tgSimulation simulation(view);

    // create the models with their controllers and add the models to the simulation
    // This constructor for TensegrityModel takes the 'debugging' flag as the
    // second argument.
    TensegrityModel* const myModel = new TensegrityModel(argv[1],false);

    // Attach a controller to the model, if desired.
    // This is a controller that interacts with a generic TensegrityModel as
    // built by the TensegrityModel file, BUT it only actually works
    // with the specific HorizontalSpine YAML file.
    // @TODO: should this throw an error when attached to a model that
    // wasn't built with the HorizontalSpine YAML file?

    // Parameters for the Horizontal Spine Controller are specified in that .h file,
    // repeated here:
    double startTime = 3.0;
    double minLength = 0.8;
    double rate = 0.25;
    std::vector<std::string> tagsToControl;
    // HF is the right horizontal set
    // HL is the bottom horizontal set maybe?
    // HB is the left horizontal set
    // HR is the top horizontal set.
    // BUT, something is wrong here. Probably Bullet's numerical problems.
    tagsToControl.push_back("HR");
    tagsToControl.push_back("HF");
    //tagsToControl.push_back("HB");
    // Call the constructor for the controller
    SpineKinematicsTestController* const controller =
      new SpineKinematicsTestController(startTime, minLength, rate, tagsToControl);
    // Attach the controller to the model. Must happen before running the
    // simulation.
    myModel->attach(controller);

    // Add the model to the world
    simulation.addModel(myModel);

    // Add sensors using the new sensing framework
    // A string prefix for the filename
    std::string log_filename = "~/NTRTsim_logs/AppSpineKinematicsTest";
    // The time interval between sensor readings:
    double timeInterval = 0.2;
    // First, create the data manager
    tgDataLogger2* myDataLogger = new tgDataLogger2(log_filename, timeInterval);
    //std::cout << myDataLogger->toString() << std::endl;
    // Then, add the model to the data logger
    myDataLogger->addSenseable(myModel);
    // Create sensor infos for all the types of sensors that the data logger
    // will create.
    //tgRodSensorInfo* myRodSensorInfo = new tgRodSensorInfo();
    tgSpringCableActuatorSensorInfo* mySCASensorInfo =
      new tgSpringCableActuatorSensorInfo();
    tgCompoundRigidSensorInfo* myCRSensorInfo = new tgCompoundRigidSensorInfo();
    // Attach the sensor infos to the data logger
    //myDataLogger->addSensorInfo(myRodSensorInfo);
    myDataLogger->addSensorInfo(mySCASensorInfo);
    myDataLogger->addSensorInfo(myCRSensorInfo);
    // Next, attach it to the simulation
    simulation.addDataManager(myDataLogger);

    simulation.run();

    // teardown is handled by delete
    return 0;
}
