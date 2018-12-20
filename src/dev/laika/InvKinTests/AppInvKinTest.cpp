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
 * @file AppInvKinTest.cpp
 * @brief Contains the definition function main() for AppInvKinTest
 * which tests arbitrary tensegrity structures' inverse kinematics in dynamics simulations.
 * @author Drew Sabelhaus
 * $Id$
 */

// This application
#include "yamlbuilder/TensegrityModel.h"
// put the controller file here
//#include "CombinedSpineControllerBending.h"
//#include "CombinedSpineControllerRotVertPositionTraj.h"
#include "InvKinTestController.h"
#include "VertConstraintForModel.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgRod.h"
#include "core/tgSimulation.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgWorld.h"
//#include "core/abstractMarker.h"
// For tracking positions:
#include "sensors/tgDataLogger2.h"
#include "sensors/tgSphereSensorInfo.h"
//#include "sensors/abstractMarkerSensorInfo.h"
// Correlating positions to cable lengths.
#include "sensors/tgSpringCableActuatorSensorInfo.h"
//DEBUGGING: see if the rod COMs, for the shoulders/hips, are the same as
// for the spheres. That would mean that bullet's COM command does the COM
// for the ENTIRE rigid, not each component, when auto-compounded!!!
#include "sensors/tgRodSensorInfo.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
// To-do: put these in as necessary for 2D structures????
// #include "core/tgWorldBulletPhysicsImpl.h" // for hinge hack
// #include "BulletDynamics/ConstraintSolver/btHingeConstraint.h" // for hinge hack
// #include "BulletDynamics/Dynamics/btDynamicsWorld.h" // for hinge hack
// The C++ Standard Library
#include <iostream>
#include <string>
#include <vector>
#include <stdexcept> // for some error handling


// A helper function for converting doubles to strings, for filenames.
// Thanks to http://www.cplusplus.com/articles/D9j2Nwbp/
template <typename T>
std::string NumberToString ( T Number )
{
  std::ostringstream ss;
  ss << Number;
  return ss.str();
}

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @param[in] argv argv[1] is the path of the YAML encoded structure
 * @param[in] argv argv[2] is the path to the inverse kinematics CSV file w/rest lengths.
 * @return 0
 */
int main(int argc, char** argv)
{
    // For this YAML parser app, need to check that an argument path was
    // passed in.
    if ((argv[1] == NULL) || (argv[2] == NULL))
    {
        // put a litthe help message.  
        std::cout << "Inverse Kinematics Testing App. Usage:" << std::endl
            << "./AppInvKinTest path_to_yaml_model path_to_invkin_csv" << std::endl << std::endl
            << "This app was called with the incorrect number of arguments." << std::endl;
        throw std::invalid_argument("Two arguments required: yaml model, and CSV of inverse kinematics inputs");
    }
    // Pick out the model and csv file.
    std::string modelPath = argv[1];
    std::string invkinCSVPath = argv[2];
  
    // create the ground and world. Specify ground rotation in radians
    const double yaw = 0.0;
    const double pitch = 0.0;
    const double roll = 0.0;
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // the world will delete this
    tgBoxGround* ground = new tgBoxGround(groundConfig);

    // We're working in centimeters, so scaling factor = 100.
    // FOR IROS 2018 DATA COLLECTION:
    const tgWorld::Config config(9.81); // gravity, dm/sec^2 is 98.1.
    // FOR IROS 2018 VISUALIZATION:
    //const tgWorld::Config config(98.1);
    // As of the commit when this comment appears, the units are still
    // a bit off. Gravity and cable force are still with s=10 (decimeters
    // and 1/10 factor on pretensions), but the length and density
    // units are for s=100 (centimeters).
    // Drew believes this is still kinematically valid, though,
    // since we're not recording force, but instead are recording
    // (effectively) the center-of-mass shift with respect to the spine.
    // For ICRA 2019 submission: no longer consider the g ~= 9.81 cases valid,
    // since the forces from the lattice/cables are scaled, and that throws
    // everything off. I really don't think NTRT simulations are OK unless
    // everything is at-scale right now. Only use g=9.81 for real data collection.
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
    TensegrityModel* const myModel = new TensegrityModel(modelPath,true);
    

    // For the inverse kinematics test, we're doing all the parsing of the 
    // CSV file in the controller.

    // For the inverse kinematics control: need
    // (a) a start time, after the simulation. Used for the structure to settle into place.
    // (b) hold time. Number of seconds after start time to apply the first input in the CSV file.
    //      This is for the structure to settle into its "starting point" for the control.
    // (c) period. How often to advance to the next control input. This is kinda like the control frequency in a ZOH sense. In sec.
    // (d) the csv file itself, containing rest lengths from the inverse kinematics from MATLAB.
    double startTime = 0.0;
    double holdTime = 0.0;
    double period = 0.1;

    // Create the controller.
    InvKinTestController* const controller = new InvKinTestController(startTime, 
        holdTime, period, invkinCSVPath);

    // attach to the model
    myModel->attach(controller);

    // Similarly, add a vertical constraint.
    // Right now this is an observer which is meh since there's no need for an onStep function,
    // but this was the most flexible implementation I could think of.
    // These CANNOT be the tags of the body in an assembly: must be the tags for the individual
    // elements within a YAML file that supplies the builder. E.g., not "vertebra" but "tetraRod."

    // create the observer
    // can use either constructor. But for now there's only one body so no need for a list.
    VertConstraintForModel* const constrainer = new VertConstraintForModel("tetraRod");

    // and attach it too.
    myModel->attach(constrainer);

    // Add the model to the world
    simulation.addModel(myModel);    
	   
    // Let's log info from the spheres (bottom of Laika's feet.)
    // EDIT: UNUSED, LOGGING OCCURS IN CONTROLLER NOW.
    
    // has to end with the prefix to the log file name.
    //std::string log_filename = "~/NTRTsim_logs/LaikaCombinedMotion";
    //double samplingTimeInterval = 0.1;
    //tgDataLogger2* myDataLogger = new tgDataLogger2(log_filename, samplingTimeInterval);
    // add the model to the data logger
    //myDataLogger->addSenseable(myModel);
    // Make it so the data logger can dispatch sphere sensors
    //abstractMarkerSensorInfo* myAbstractMarkerSensorInfo = new abstractMarkerSensorInfo();
    // Correlating the cable lengths to foot positions.
    //tgSpringCableActuatorSensorInfo * mySCASensorInfo = new tgSpringCableActuatorSensorInfo();
    //tgSphereSensorInfo* mySphereSensorInfo = new tgSphereSensorInfo();
    //DEBUGGING: rods too
    //tgRodSensorInfo* myRodSensorInfo = new tgRodSensorInfo();
    //myDataLogger->addSensorInfo(myAbstractMarkerSensorInfo);
    //myDataLogger->addSensorInfo(mySCASensorInfo);
    //myDataLogger->addSensorInfo(mySphereSensorInfo);
    //myDataLogger->addSensorInfo(myRodSensorInfo);
    // Add the data logger to the simulation.
    //simulation.addDataManager(myDataLogger);
    
    // Finally, run the simulation.
    simulation.run();

    // teardown is handled by delete
    return 0;
}
