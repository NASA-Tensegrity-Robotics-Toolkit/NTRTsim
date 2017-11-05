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
 * @file T6RollingControllerPrism.cpp
 * @brief Implementation of the rolling controller.
 * @author Brian Cera adapted from code by Edward Zhu
 * @version 1.0.0
 * $Id$
 */

// This module
#include "SimulatedSensors.h"
#include "core/abstractMarker.h" 
// The C++ Standard Library
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include <algorithm>
#include <math.h>
// Boost Matrix Library
#include "numeric/ublas/matrix.hpp"
#include <numeric/ublas/assignment.hpp>
#include <numeric/ublas/operation.hpp>
//Boost Vector Library
#include <numeric/ublas/vector.hpp>
// Utility Library
#include "../utility.hpp"

using namespace boost::numeric::ublas;

namespace{
  
  double sf = 30; //TODO: pass sf in the constructor
  double worldTime = 0;
  
}

SimulatedSensors::SimulatedSensors()
{
}

SimulatedSensors::~SimulatedSensors()
{
}

void SimulatedSensors::onSetup(PrismModel& subject)
{
 
  // Retrieve rods from model
  rods = subject.getAllRods();
  //std::cout << "onSetup: Number of rods: " << rods.size() << std::endl;

  // Convert from tgRod objects to btRigidBodyT objects
  for (size_t i = 0; i < rods.size(); i++) {
    tgRod* rod = rods[i];
    btRigidBody* rodBody = rod->getPRigidBody();
    rodBodies.push_back(rodBody);
  }

  // Retrieve normal vectors from model
  normVects = subject.getNormVects();

  // Obtain Cables for other potential sensors (later)
  m_controllers.clear();
  actuators = subject.getAllActuators();
  cables = subject.getAllCables();	
  //std::cout << "onSetup: Number of actuators: " << actuators.size() << std::endl;
  for (size_t i = 0; i < actuators.size(); i++) {
    tgBasicActuator* const pActuator = actuators[i];
    assert(pActuator != NULL);
    tgBasicController* m_lenController = new tgBasicController(pActuator, restLength);
    m_controllers.push_back(m_lenController);
  }

  // Find the rest length and start length of the cables
  restLength = actuators[0]->getRestLength();
  startLength = actuators[0]->getStartLength();

}

void SimulatedSensors::onStep(PrismModel& subject, double dt)
{
  /////////////////////////////////////Optional printouts ///////////////////////////////////////////
  //std::cout << std::endl;
  //std::cout << "OverallTime: " << worldTime << std::endl;
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  
  worldTime += dt;
  if (dt <= 0.0) {
    throw std::invalid_argument("onStep: dt is not positive");
  }

  //Orientation Sensor
  double frequency = 100; //Sensor update frequency [Hz]
  if(fmod(worldTime,1.0/frequency)<=dt){
    vector<double> Input = OrientationSensor();
    std::cout << "Simulated_Orientation_Sensors, ";
    for(size_t i=0; i<Input.size() ; i++){
      std::cout << Input(i) << ", ";
    }
    std::cout << std::endl;
 }

}


vector<double> SimulatedSensors::OrientationSensor()
{ 
  // Initialize vector of phi angles (angles between rod directions and +Y axis)
  vector<double> phiInput(6);
  vector<double> relthetaInput(5);
  double rod1theta;
  
  //rod theta and phi angles, theta W.R.T. x-axis
  for(size_t i=0; i<6; i++){
    btTransform worldTrans = rodBodies[i]->getWorldTransform();
    btMatrix3x3 robot2world = worldTrans.getBasis();
    btVector3 rodDir = robot2world*btVector3(0,1,0);
    
    //calculate relative theta
    if(i==0){
      rod1theta = atan2(-rodDir.z(),rodDir.x());
      if(rod1theta<0)
	rod1theta += 2*M_PI;
    }
    else if(i!=0){
      double theta = atan2(-rodDir.z(),rodDir.x());
      theta -= rod1theta;
      if(theta<0)
	theta += 2*M_PI;
      relthetaInput(i-1) = theta;
    }

    //calculate phi
    double angle = rodDir.angle(btVector3(0,1,0));
    phiInput(i) = angle;
  }

  //concatenate input feature vector
  vector<double> Input(11);
  Input <<= phiInput, relthetaInput;
  //std::cout << "Input size: " << Input.size() << std::endl;

  return Input;
}
