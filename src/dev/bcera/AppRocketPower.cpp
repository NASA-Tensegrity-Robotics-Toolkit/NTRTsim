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
 * @file AppThruster.cpp
 * @brief Contains the definition function main() for the thruster
 * application.
 * $Id$
 */

// This application
#include "RPModel.h"
#include "RPThruster.h" //##### BCera - Controller Added in Later
#include "RPTensionController.h" //Controller added^^
#include "RPLengthController.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/terrain/tgHillyGround.h"
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <iostream>
#include <unistd.h>

void simulate(tgSimulation *simulation);


/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */
int main(int argc, char** argv)
{
  std::cout << "AppThruster" << std::endl;
  double sf = 3; // scaling factor -  match with model file

  // First create the ground and world
  // Determine type of the ground to be used
  // Uncomment either 1. Flat ground or 2. Hilly ground
  
  // ------------------ 1. Flat ground --------------------------------------------------------------------------------------------------
  
  // Determine the angle of the ground in radians. All 0 is flat
  const double yaw = 0.0;
  const double pitch = 0.0; //0*M_PI/15.0;
  const double roll = 0.0;
  const double friction = 0.5;
  const double restitution = 0.0;
  const double worldSizeX = 15000.0;
  const double worldSizeY = 1.5;
  const double worldSizeZ = 15000.0;    
   
  const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll),friction,restitution,btVector3(worldSizeX,worldSizeY,worldSizeZ));
  // the world will delete this
  tgBoxGround* ground = new tgBoxGround(groundConfig);
   
  const tgWorld::Config config
    (
     // Note, by changing the setting below from 981 to 98.1, we've
     // scaled the world length scale to decimeters not cm.
   
     // Gravity, in cm/sec^2. Use this to adjust length scale of world.			
     // 9.81*sf // use this to simulate gravity of the Earth
     //1.622*sf // use this to simulate gravity of the Moon
     0 // use this to simulate absence of gravity
     );
  
  // ------------------ End of Flat ground -------------------------------------------------------------------------------------------------
  
  // ------------------ 2. Hilly ground ----------------------------------------------------------------------------------------------------
  /*
    const tgWorld::Config config(1.622*sf); // gravity, cm/sec^2

    btVector3 eulerAngles = btVector3(0.0, 0.0, 0.0);
    btScalar friction = 0.5;
    btScalar restitution = 0.0;
    btVector3 size = btVector3(500.0, 0.5, 500.0);
    btVector3 origin = btVector3(0.0, 0.0, 0.0);
    size_t nx = 500; // use 500 with graphic turned on; use 2500 with graphic turned off
    size_t ny = 50; // use 50 with graphic turned on; use 400 with graphic turned off
    double margin = 0;
    double triangleSize = 10.0;
    double waveHeight = 10.0;
    double offset = 0.0;
    tgHillyGround::Config groundConfig(eulerAngles, friction, restitution,
    size, origin, nx, ny, margin, triangleSize,
    waveHeight, offset);		
    tgHillyGround* ground = new tgHillyGround(groundConfig);
  */
  
  // ----------------- End of Hilly Ground -------------------------------------------------------------------------------------------------

  // Create world
  tgWorld world(config, ground);
  // Create world with no ground;
  //tgWorld world(config);

  // Second create the view
  const double timestep_physics = 0.0001; // Seconds
  const double timestep_graphics = 1.f/60.f; // Seconds

  // Turn on/off GUI-----------------------------------------------------------
  //use GUI and graphics for manual 1-by-1 iteration
  tgSimViewGraphics view(world, timestep_physics, timestep_graphics); // turn on graphics

  //turn off graphics and GUI for self iterating process with randomization 
  //tgSimView view(world, timestep_physics, timestep_graphics); // turn off graphics
  //----------------------------------------------------------------------------

  // Third create the simulation
  tgSimulation simulation(view);

  // Fourth create the models with their controllers and add the models to the
  // simulation
  RPModel* const myModel = new RPModel();

  
  //uncomment for length controllers
  RPLengthController* const pTC = new RPLengthController();
  myModel -> attach(pTC);
  
  //uncomment for tension controllers
  //RPTensionController* const pTC = new RPTensionController();
  //myModel ->attach(pTC);
  
  //Attach the thruster controller
  //RPThruster* const pTC2 = new RPThruster();
  //myModel->attach(pTC2);
  
  simulation.addModel(myModel); 


  

  /*
  //uncomment for loop and ints for self iterating process with
  //randomized controller

  int nRuns = 12;
  int nSteps = 600;
  for (int i=0; i<nRuns; i++) {
  simulation.run(nSteps);
  std::cout << "Simulations complete:"<< i+1 << std::endl;
  std::cout << "Reseting..." << std::endl;
  simulation.reset(); // <-this doesn't work with the tension controller...
  }
  */

  simulate(&simulation);

  //Teardown is handled by delete, so that should be automatic
  return 0;
}

void simulate(tgSimulation *simulation) {
  int nEpisodes = 1; // Number of episodes ("trial runs")
  int nSteps = 260000; // Number of steps in each episode, 60k is 100 seconds (timestep_physics*nSteps)
  for (int i=0; i<nEpisodes; i++) {
    simulation->run(nSteps);
    simulation->reset();
    std::cout << "Simulations complete:"<< i+1 << std::endl;
    std::cout << "Reseting..." << std::endl;
  }
}
