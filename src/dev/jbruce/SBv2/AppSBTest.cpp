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
 * @file AppSBTest.cpp
 * @brief Basic Testing of YAML and SB model building. Should be
 * depricated once the offical model is done
 * $Id$
 */

// This application
#include "yamlbuilder/TensegrityModel.h"
#include "StepwiseController.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/terrain/tgImportGround.h"
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
#include <fstream>


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
    //Option 1: Uncomment to implement flat ground environment (must comment out STL importer below)
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  /* 
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // the world will delete this
    tgBoxGround* ground = new tgBoxGround(groundConfig);
  */
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //Option 2: Uncomment below to import parsed STL file (must comment out flat ground above)
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Set ground parameters
  btVector3 orientation = btVector3(yaw, pitch, roll);
  const double friction = 0.9;
  const double restitution = 0.0;
  btVector3 origin = btVector3(0.0, 0.0, 0.0);
  const double margin = 0;//0.05;
  const double offset = 0;
  const double scalingFactor = 500; //sf*1000/63;
  int Interp = 0;
  bool twoLayer = false;
  
  // Configure ground characteristics for simulation
  const tgImportGround::Config groundConfig(orientation, friction, restitution,
					    origin, margin, offset, scalingFactor,Interp,twoLayer);
  
  // Get filename from argv
  //std::string filename_in = "./LunarScape_mission.txt";
  // std::string filename_in = "./STL_output.txt" ;
  std::string filename_in = "/home/jonathan/test.txt" ;

  
  // Check filename
  if (filename_in.find(".txt") == std::string::npos) {
    std::cout << "Incorrect filetype, input file should be a .txt file" << std::endl;
    exit(EXIT_FAILURE);
  }
  
  //Create filestream
  std::fstream file_in;
  // Open filestream
  file_in.open(filename_in.c_str(), std::fstream::in);
  // Check if input file opened successfully
  if (!file_in.is_open()) {
    std::cout << "Failed to open input file" << std::endl;
    exit(EXIT_FAILURE);
  }
  else {
    std::cout << "Input file opened successfully" << std::endl;
  }
  tgImportGround* ground = new tgImportGround(groundConfig, file_in);
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    const tgWorld::Config config(98.1); // gravity, dm/sec^2
    tgWorld world(config, ground);

    // create the view
//    const double timestep_physics = 0.0001; // seconds
    const double timestep_physics = 0.001;
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
    // built by the TensegrityModel file

    // Parameters for the SingleCableController or StewiseController are specified in that .h file,
    // repeated here:
    double startTime = 5.0;
    double stepTime = 2.0;
    double minLength = 0.1;
    double rate = 3.5;
    std::vector<std::string> tagsToControl;
    //tagsToControl.push_back("m18"); // Tag located in SBv2_yaml_files/SBv2_model.yaml line: 28

    tagsToControl.push_back("m10");
    tagsToControl.push_back("m01");
    tagsToControl.push_back("m21");
    tagsToControl.push_back("m15");
    tagsToControl.push_back("m08");
    tagsToControl.push_back("m20");

    // tagsToControl.push_back("m01");
    // tagsToControl.push_back("m02");
    // tagsToControl.push_back("m03");
    // tagsToControl.push_back("m04");
    // tagsToControl.push_back("m05");
    // tagsToControl.push_back("m06");
    // tagsToControl.push_back("m07");
    // tagsToControl.push_back("m08");
    // tagsToControl.push_back("m09");
    // tagsToControl.push_back("m10");
    // tagsToControl.push_back("m11");
    // tagsToControl.push_back("m12");
    // tagsToControl.push_back("m13");
    // tagsToControl.push_back("m14");
    // tagsToControl.push_back("m15");
    // tagsToControl.push_back("m16");
    // tagsToControl.push_back("m17");
    // tagsToControl.push_back("m18");
    // tagsToControl.push_back("m19");
    // tagsToControl.push_back("m20");
    // tagsToControl.push_back("m21");
    // tagsToControl.push_back("m22");
    // tagsToControl.push_back("m23");
    // tagsToControl.push_back("m24");

    // Call the constructor for the controller
    StepwiseController* const controller =
      new StepwiseController(startTime, stepTime, minLength, rate, tagsToControl);

    // Attach the controller to the model. Must happen before running the
    // simulation.
    myModel->attach(controller);

    // Add the model to the world
    simulation.addModel(myModel);

    simulation.run();

    // teardown is handled by delete
    return 0;
}
