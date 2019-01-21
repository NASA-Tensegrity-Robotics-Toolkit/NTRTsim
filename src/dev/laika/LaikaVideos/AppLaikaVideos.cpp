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
 * @file AppLaikaVideos.cpp
 * @brief Contains the definition function main() for AppLaikaVideos
 * which is used to make nice videos for presentations and such.
 * @author Drew Sabelhaus
 * $Id$
 */

// This application
#include "yamlbuilder/TensegrityModel.h"
// put the controller file here
// we're going to directly re-use the one from the Laika foot-lifting experiment,
// which is in another directory.
#include "../v0.2_combined/CombinedSpineControllerBending.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/terrain/tgEmptyGround.h"
#include "core/tgModel.h"
#include "core/tgRod.h"
#include "core/tgSimulation.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgWorld.h"
//#include "core/abstractMarker.h"
// For tracking positions:
// #include "sensors/tgDataLogger2.h"
// #include "sensors/tgCompoundRigidSensorInfo.h"
//#include "sensors/abstractMarkerSensorInfo.h"
// Correlating positions to cable lengths.
// #include "sensors/tgSpringCableActuatorSensorInfo.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
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
    if ((argv[1] == NULL))
    {
        // put a litthe help message.  
        std::cout << "Laika Videos app. Usage:" << std::endl
            << "./AppLaikaVideos path_to_yaml_file" << std::endl << std::endl
            << "This app was called with the incorrect number of arguments." << std::endl;
        throw std::invalid_argument("Pass in the YAML file.");
    }
    // Pick out the model and csv file.
    std::string modelPath = argv[1];
  
    // create the ground and world. Specify ground rotation in radians
    const double yaw = 0.0;
    const double pitch = 0.0;
    const double roll = 0.0;
    //const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // the world will delete this
    //tgBoxGround* ground = new tgBoxGround(groundConfig);

    // We're working in centimeters, so scaling factor = 100.
    // FOR IROS 2018 DATA COLLECTION:
    const tgWorld::Config config(98.1); // gravity, dm/sec^2 is 98.1.
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
    //tgWorld world(config, ground);
    tgWorld world(config, new tgEmptyGround());

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
    TensegrityModel* const myModel = new TensegrityModel(modelPath, true);
    //TensegrityModel* const myModel = new TensegrityModel(modelPath, false);

    // For the videos, just attach however many of the combined motion controllers we want.
    // These controllers each pull in a cable by a certain amount, at a given rate, starting at a certain time.
    // Horizontal bending controller 1
    // double startTimeBend = 0.0;
    // double minLength = 1.0; // 1.0 is 100%, no bending
    // double rate = 0.0;
    std::vector<std::string> tagsToControl;
    tagsToControl.push_back("HR");

    // Parameters for the Horizontal Spine Controller are specified in that .h file,
    // repeated here:
    double startTime = 5.0;
    double minLength = 0.8;
    double rate = 0.25;
    // Call the constructor for the controller
    CombinedSpineControllerBending* const controller =
      new CombinedSpineControllerBending(startTime, minLength, rate, tagsToControl);
    // Attach the controller to the model. Must happen before running the
    // simulation.
    myModel->attach(controller);
    
    // Add the model to the world
    simulation.addModel(myModel);    
	   
    // Since we just want to look at the graphical output,
    // no need to track anything.
    
    // Finally, run the simulation.
    simulation.run();

    // teardown is handled by delete
    return 0;
}
