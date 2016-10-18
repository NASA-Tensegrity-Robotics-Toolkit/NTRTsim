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
 * @file AppPrismModel.cpp
 * @brief Contains the definition function main() for the Three strut
 * tensegrity prism example application
 * @author Brian Tietz
 * $Id$
 */

// This application
#include "PrismModel.h"
//#include "RPThruster.h"
#include "controllers/T6RollingControllerPrism.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/terrain/tgImportGround.h"
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#define PI 3.14159
/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */
int main(int argc, char** argv)
{
  double sf = 100; //Scaling Factor - match with model and controller files
   
  // First create the ground and world. Specify ground rotation in radians
  const double yaw = 0.0;
  const double pitch = -15*PI/180;//0.1;//0.2;
  const double roll = 0.0;

  
  const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
  // the world will delete this
  tgBoxGround* ground = new tgBoxGround(groundConfig);

  /*
  // Import Ground
  // Set ground parameters
  btVector3 orientation = btVector3(yaw, pitch, roll);
  const double friction = 0.5;
  const double restitution = 0.;
  btVector3 origin = btVector3(0.0, 0.0, 0.0);
  const double margin = 0.05;
  const double offset = 0.5;
  const double scalingFactor = 100;
  // Configure ground characteristics
  const tgImportGround::Config groundConfig(orientation, friction, restitution,
					    origin, margin, offset, scalingFactor);
  // Get filename from argv
  //std::string filename_in = argv[1];
  std::string filename_in = "./lunarscape_mission.txt";
  //std::string filename_in = "./LunarScape_crater2Ex.txt";
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
  */

  double gravity = 9.81*sf;
  const tgWorld::Config config(gravity); // gravity, cm/sec^2
  tgWorld world(config, ground);

  // Second create the view
  const double timestep_physics = 0.001; // seconds
  const double timestep_graphics = 1.f/60.f; // seconds
  tgSimViewGraphics view(world, timestep_physics, timestep_graphics); //turn on graphics
  //tgSimView view(world, timestep_physics, timestep_graphics); // turn off graphics for faster data logging

  // Third create the simulation
  tgSimulation simulation(view);

  // Fourth create the models with their controllers and add the models to the
  // simulation
  PrismModel* const myModel = new PrismModel();

  // Define path for controller
  int *ptr;
  //int path[] = {2, 15, 13, 0, 5, 7, 10}; // Repeat unit is [15, 13, 0, 5, 7, 10]
  int path[] = {10, 8, 5, 0, 2, 15};
  int pathSize = sizeof(path)/sizeof(int);
  ptr = path;

  //Create Active Thruster
  //RPThruster* const thrust_control = new RPThruster();
  //myModel->attach(thrust_control);

  const T6RollingController::Config controllerConfig(gravity, "path", ptr, pathSize);
  T6RollingController* const rollingController = new T6RollingController(controllerConfig);
  //myModel->attach(rollingController);
  
    
  // Add the model to the world
  simulation.addModel(myModel);
    
  simulation.run(100000);

  //Teardown is handled by delete, so that should be automatic
  return 0;
}
