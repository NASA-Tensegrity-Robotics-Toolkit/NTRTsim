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
 * @file AppLaikaTorsion.cpp
 * @brief Contains the definition function main() for AppLaikaTorsion
 * which builds a horizontal spine structure defined in YAML and which
 * has a controller to do torsional movement
 * @author Andrew Sabelhaus, Ankita Joshi, Robel Teweldebirhan
 * $Id$
 */

// This application
#include "yamlbuilder/TensegrityModel.h"
#include "LaikaTorsionController.h"
#include "LaikaReverseTorsionController.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgSimulation.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgWorld.h"
// this tgDataLogger2 library
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

    const tgWorld::Config config(98.1); // gravity, dm/sec^2
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

    // Parameters for the Laika Torsion Controller are specified in that .h file,
    // repeated here:
    double startTime = 2.0;
    double minLength = 0.4;
    double maxLength = 1.6;
    double rate = 1;
    std::vector<std::string> tagsToControl;
    // HF is the right horizontal set
    // HL is the bottom horizontal set maybe?
    // HB is the left horizontal set
    // HR is the top horizontal set.
    // BUT, something is wrong here. Probably Bullet's numerical problems.
    //tagsToControl.push_back("HR");
    //tagsToControl.push_back("SFR");
    //tagsToControl.push_back("HB");
// The below controlers should only be used in spine made up of Spider structure.
    //tagsToControl.push_back("HR");
    //tagsToControl.push_back("HRU");
    tagsToControl.push_back("VU");
    //tagsToControl.push_back("HLU");
    //tagsToControl.push_back("HL");
    //tagsToControl.push_back("HLL");
    //tagsToControl.push_back("VL");
    //tagsToControl.push_back("HRL");
    //tagsToControl.push_back("TCR");
    //tagsToControl.push_back("TCL");
    //tagsToControl.push_back("SFR");
    //tagsToControl.push_back("SFL");
    //tagsToControl.push_back("WCR");
    //tagsToControl.push_back("WCL");
    // Call the constructor for the controller
    LaikaTorsionController* const controller =
      new LaikaTorsionController(startTime, minLength, rate, tagsToControl);
    // Attach the Reverse controller to the model. Must happen before running the
    // simulation.
    std::vector<std::string> tagsToReverseControl;
    // HF is the right horizontal set
    // HL is the bottom horizontal set maybe?
    // HB is the left horizontal set
    // HR is the top horizontal set.
    // BUT, something is wrong here. Probably Bullet's numerical problems.
    //tagsToReverseControl.push_back("HR");
    //tagsToReverseControl.push_back("SFR");
    //tagsToReverseControl.push_back("HB");
// The below controlers should only be used in spine made up of Spider structure.
    //tagsToReverseControl.push_back("HR");
    //tagsToReverseControl.push_back("HRU");
    //tagsToReverseControl.push_back("VU");
    //tagsToReverseControl.push_back("HLU");
    //tagsToReverseControl.push_back("HL");
    //tagsToReverseControl.push_back("HLL");
    //tagsToReverseControl.push_back("VL");
    //tagsToReverseControl.push_back("HRL");
    //tagsToReverseControl.push_back("TCR");
    //tagsToReverseControl.push_back("TCL");
    //tagsToReverseControl.push_back("WCR");
    //tagsToReverseControl.push_back("WCL");
    //tagsToReverseControl.push_back("SFR");
    //tagsToReverseControl.push_back("SFL");
    // Call the constructor for the controller
    LaikaReverseTorsionController* const Reversecontroller =
      new LaikaReverseTorsionController(startTime, maxLength, rate, tagsToReverseControl);
    // Attach the controller to the model. Must happen before running the
    // simulation.
    myModel->attach(controller);
    myModel->attach(Reversecontroller);
    // Add the model to the world
    simulation.addModel(myModel);

    // Add sensors using the new sensing fameworks
    // A string Prefix for the filename 
    // Data Logger to rteweldebirhan/NTRT_logs
    std::string log_filename = "~/NTRT_logs/Laika";
    // The time Interval Between sensor readings 
    double timeInterval = 0.2;
    // Creating Data Manger
    tgDataLogger2* myDataLogger = new tgDataLogger2(log_filename, timeInterval);
    // std::cout <<myDataLogger->toString()<< std::endl;
    //Adding the model to the data logger
    myDataLogger->addSenseable(myModel);
    // Creating sensor infos for all the types of sensors that the data logger will create.
    tgRodSensorInfo* myRodSensorInfo = new tgRodSensorInfo();
    tgSpringCableActuatorSensorInfo* mySCASensorInfo = new tgSpringCableActuatorSensorInfo();
    tgCompoundRigidSensorInfo* myCRSensorInfo = new tgCompoundRigidSensorInfo();
    // Attaching the sensor infos to the data logger
    myDataLogger->addSensorInfo(myRodSensorInfo);
    myDataLogger->addSensorInfo(mySCASensorInfo);
    myDataLogger->addSensorInfo(myCRSensorInfo);
    // Attaching it to the simulation
    simulation.addDataManager(myDataLogger);

    simulation.run();

    // teardown is handled by delete
    return 0;

}
