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

#ifndef SIMULATED_SENSORS_H
#define SIMULATED_SENSORS_H

/**
 * @file SimulatedSensors.h
 * @brief Contains the definition of Simulated Sensors. Outputs to cout for piping to a ROS node
 * @author Brian Cera
 * @version 1.0.0
 * $Id$
 */

// This Library
#include "core/tgObserver.h"
#include "core/tgRod.h"
#include "controllers/tgBasicController.h"

// The Model
#include "../PrismModel.h"

// The Bullet Physics Library
#include "LinearMath/btVector3.h"

// The C++ Standard Library
#include <vector>
#include <string>

//Boost Vector Library
#include <numeric/ublas/vector.hpp>


// Forward declarations
class PrismModel;

/**
 * A controller which allows for rolling to a goal triangle or rolling with
 * dead reckoning
 */

class SimulatedSensors : public tgObserver<PrismModel>
{
 public:
  /**
   * Configuration structure for setting the mode and goal of the rolling
   * controller
   */
  
  /**
   * Constructor, allows a user to specify their own config
   */
  SimulatedSensors();

  /**
   * Destructor
   */
  virtual ~SimulatedSensors();

  /**
   * Select controller mode based on configuration. Define the normal vectors for 
   * all icosahedron faces as well as the adjacency matrix.
   * @param[in] subject - the model that the controller attaches to
   */
  virtual void onSetup(PrismModel& subject);

  /**
   * Run the controller.
   * @param[in] subject - the model that the controller attaches to
   * @param[in] dt - the physics time step from the app file
   */
  virtual void onStep(PrismModel& subject, double dt);

  boost::numeric::ublas::vector<double> OrientationSensor();


 private:

  // Vector of rigid body objects
  std::vector<btRigidBody*> rodBodies;

  // A vector to hold all normal vectors
  std::vector<btVector3> normVects;

  // Vectors to hold actuators and rods
  std::vector<tgBasicActuator*> actuators;
  std::vector<tgBasicActuator*> cables; //all cables including passive inner cables
  std::vector<tgRod*> rods;
  //std::vector<abstractMarker> markers;
  std::vector<btRigidBody*> tank;

  // Vector to hold controllers for the cables
  std::vector<tgBasicController*> m_controllers;
	
  // Rest length and start length of cables
  double restLength;
  double startLength;

};

#endif
