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
 * @author Brian Cera
 * $Id$
 */

// This application
#include "PrismModel.h"
#include "Marker.h"
#include "RPThruster.h"
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

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */

int main(int argc, char** argv)
{
  double sf = 30; //Scaling Factor - match with model and controller files
   
  // First create the ground and world. Specify ground rotation in radians
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
  const double friction = 1.0;
  const double restitution = 0.0;
  btVector3 origin = btVector3(0.0, 0.0, 0.0);
  const double margin = 35;//0.05;
  const double offset = 0.5;
  const double scalingFactor = sf*1000/63;
  int Interp = 0;
  bool twoLayer = false;
  
  // Configure ground characteristics for simulation
  const tgImportGround::Config groundConfig(orientation, friction, restitution,
					    origin, margin, offset, scalingFactor,Interp,twoLayer);
  
  // Get filename from argv
  std::string filename_in = "./LunarScape_mission.txt";
  
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
 
  double gravity = 1.618*sf; //moon gravity
  const tgWorld::Config config(gravity); 
  tgWorld world(config, ground);

  //Second create the view
  const double timestep_physics = 0.00075; // seconds
  const double timestep_graphics = 1.f/60.f; // seconds
  tgSimViewGraphics view(world, timestep_physics, timestep_graphics); //turn on graphics (option 1)
  //tgSimView view(world, timestep_physics, timestep_graphics); // uncomment to turn off graphics for faster data logging (option 2)

  //Third create the simulation
  tgSimulation simulation(view);

  //Define target destination
  //btVector3 target = btVector3(1000, 0, -2000);
  //btVector3 target = btVector3(1000*sf, 0, -1000*sf);
  //btVector3 target = btVector3(450*sf, 0, -10*sf);
  btVector3 target = btVector3(150*sf, 0, -100*sf);
  //btVector3 target = btVector3(700*sf, 0, -10*sf);
  
  //Fourth create the model with its controllers and add the models to the simulation 
  PrismModel* const myModel = new PrismModel();

  //Place a model to signify target location
  btVector3 marker_target = target;
  marker_target.setY(100);
  MarkerModel* const marker = new MarkerModel(marker_target);
    
  //Create Active Thruster
  RPThruster* const thrust_control = new RPThruster(2,3,1,target); //robotstate=2 activates thruster controller, goes to state 3 if robot is within distance threshold, else state 1
  myModel->attach(thrust_control);

  //Robot Starts in State 1
  const T6RollingController::Config controllerConfig1(gravity, "face", 0, 1, 2); //reorientation controller is state 1 -> state 2 thrust control
  const T6RollingController::Config controllerConfig2(gravity, "dr", target, 3, 1); //dead reckoning is state 3 -> state 1 reorientation

  //Attach both rolling controllers (dead reckoning and reorientation rolling)
  T6RollingController* const rollingController1 = new T6RollingController(controllerConfig1);
  myModel->attach(rollingController1);
  T6RollingController* const rollingController2 = new T6RollingController(controllerConfig2);
  myModel->attach(rollingController2);
    
  // Add the models to the world
  simulation.addModel(marker);
  simulation.addModel(myModel);
    
  simulation.run(100000); //if graphics is turned off, will only run for N timesteps; simulation.run(N)

  //Teardown is handled by delete, so that should be automatic
  return 0;
}
