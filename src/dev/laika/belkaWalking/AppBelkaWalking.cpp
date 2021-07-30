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
 * @file AppBelkaWalking.cpp
 * @brief Contains the definition function main() for AppBelkaWalking. 
 * which builds a horizontal spine structure defined in YAML (under the laika branch)
 * in the geometry needed for a hand-tuned gait.
 * @author Edward Zhu, Andrew Sabelhaus
 * $Id$
 */

// This application
// #include "yamlbuilder/TensegrityModel.h"
#include "BelkaWalkingController.h"
#include "BelkaWalkingFileController.h"
#include "BelkaWalkingModel.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgSimulation.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgWorld.h"
// For tracking positions:
#include "sensors/tgDataLogger2.h"
#include "sensors/tgSphereSensorInfo.h"
#include "sensors/tgCompoundRigidSensorInfo.h"
#include "sensors/tgSpringCableActuatorSensorInfo.h"
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
 * @return 0
 */
int main(int argc, char** argv)
{
    // Normally, for an App that uses the YAML parser, we would need to check
    // the number of arguments passed in. However, since we'll be hard-coding the
    // path to the YAML file in the app, no arguments are desired.
    if (argv[1] != NULL)
    {
       throw std::invalid_argument("This app does not take in a YAML file, it's hard-coded for ease, or edit the string path to the YAML file in this app.");
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
    const double timestep_physics = 0.00005;
    // const double timestep_physics = 0.0001; // seconds
    // const double timestep_physics = 0.001;
    const double timestep_graphics = 1.f/60.f; // seconds

    // Two different simulation views. Use the graphical view for debugging...
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);
    // ...or the basic view for running DRL.
    //tgSimView view(world, timestep_physics, timestep_graphics);

    // create the simulation
    tgSimulation simulation(view);

    // create the models with their controllers and add the models to the simulation
    // This constructor for TensegrityModel takes the 'debugging' flag as the
    // second argument.
    // Note that we hard-code the path to the correct YAML file here!
    // If the BelkaWalkingModel tgModel is not passed the correct YAML file, it
    // will segfault. Drew doesn't have time to code the right checks just yet...

    // NOTE: change this path to the correct one on your computer.
    // Otherwise, this app won't run.
    std::cout << "WARNING: Be sure to change the hard-coded path to the YAML file in AppBelkaWalking, which will depend on the folder path on your computer." << std::endl;
    
    // std::string model_path("/home/drew/repositories/NTRTsim/src/dev/laika/belkaWalking/BelkaWith1DOFLegs.yaml");
    std::string model_path("/home/drew/repositories/NTRTsim/src/dev/laika/belkaWalking/BelkaTranslated.yaml");
    //std::string model_path("/home/drew/repositories/NTRTsim/src/dev/laika/BaseStructuresLaika/StickLegs.yaml");
    std::cout << "Setting up the BelkaWalkingModel..." << std::endl;
    // BelkaWalkingModel* const myModel = new BelkaWalkingModel(model_path.c_str(),false);
    TensegrityModel* const myModel = new BelkaWalkingModel(model_path.c_str(),false);

    // Attach a controller to the model, if desired.
    // This is a controller that interacts with a generic TensegrityModel as
    // built by the TensegrityModel file, BUT it only actually works
    // with the specific HorizontalSpine YAML file.
    // @TODO: should this throw an error when attached to a model that
    // wasn't built with the HorizontalSpine YAML file?

    /*
    // Parameters for the Horizontal Spine Controller are specified in that .h file,
    // repeated here:
    double startTime = 10.0;
    double minLength = 0.8;
    double rate = 0.25;
    */

    // Call the constructor for the controller. Tags are now hard-coded.
    // For keyboard control only:
    // BelkaWalkingController* const controller = new BelkaWalkingController();
    // Now, reading control inputs from a CSV file:
    // std::string input_traj("~/repositories/NTRTsim/src/dev/laika/belkaWalking/control_trajectories/belka_acbd_20deg_2020-10-11.csv");
    std::string input_traj("~/repositories/NTRTsim/src/dev/laika/belkaWalking/control_trajectories/belka_withmirror_20deg_2020-12-4.csv");
    // std::string input_traj("~/repositories/NTRTsim/src/dev/laika/belkaWalking/control_trajectories/belka_acbd_longer_20deg_2020-12-31.csv");
    // std::string input_traj("~/repositories/NTRTsim/src/dev/laika/belkaWalking/control_trajectories/belka_bdac_longer_20deg_2020-12-31.csv");
    BelkaWalkingFileController* const controller = new BelkaWalkingFileController(input_traj);

    // Attach the controller to the model. Must happen before running the
    // simulation.
    myModel->attach(controller);

    // Add the model to the world
    simulation.addModel(myModel);

    // has to end with the prefix to the log file name.
    std::string log_filename = "~/NTRTsim_logs/AppBelkaWalking_";
    double samplingTimeInterval = 0.1;
    tgDataLogger2* myDataLogger = new tgDataLogger2(log_filename, samplingTimeInterval);
    // add the model to the data logger
    myDataLogger->addSenseable(myModel);
    // Make it so the data logger can dispatch sphere sensors
    //abstractMarkerSensorInfo* myAbstractMarkerSensorInfo = new abstractMarkerSensorInfo();
    // Correlating the cable lengths to foot positions.
    tgSpringCableActuatorSensorInfo* mySCASensorInfo = new tgSpringCableActuatorSensorInfo();
    // for the compound bodies (this should be all vertebrae)
    tgCompoundRigidSensorInfo* myCRSensorInfo = new tgCompoundRigidSensorInfo();
    tgSphereSensorInfo* mySphereSensorInfo = new tgSphereSensorInfo();
    //DEBUGGING: rods too
    //tgRodSensorInfo* myRodSensorInfo = new tgRodSensorInfo();
    //myDataLogger->addSensorInfo(myAbstractMarkerSensorInfo);
    myDataLogger->addSensorInfo(mySCASensorInfo);
    myDataLogger->addSensorInfo(myCRSensorInfo);
    myDataLogger->addSensorInfo(mySphereSensorInfo);
    //myDataLogger->addSensorInfo(myRodSensorInfo);
    // Add the data logger to the simulation.
    simulation.addDataManager(myDataLogger);

    // Finally, run the simulation.
    simulation.run();

    // teardown is handled by delete
    return 0;
}

