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

#ifndef RP_THRUSTER_H
#define RP_THRUSTER_H
#include <math.h>

/**
 * @file RPThruster.h
 * @brief Contains the definition of class RPThruster.
 * @author Brian Cera based on code by Kyunam Kim
 * @version 1.0.0
 * $Id$
 */

// This library
#include "core/tgObserver.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <vector>
#include <iostream>
#include <fstream>

// Forward declarations
class PrismModel;

/**
 * A controller that applies thrust to a center payload.
 */
class RPThruster : public tgObserver<PrismModel>
{
public:
	
  /**
   * Construct a RPThruster.
   * @param[in] activate - case number for robot state for which this controller is active.
   * @param[in] transfer - case number for robot state which this controller leads to when finished
   */
  
  // Note that currently this is calibrated for decimeters.
  RPThruster(int activate, int transfer, int transfer2, btVector3 targetLocation);
    
  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~RPThruster() { }

  /**
   * On setup, this controller does nothing.
   * @param[in] subject - the RPModel that is being controlled. Must
   * have a list of allMuscles populated
   */
  virtual void onSetup(PrismModel& subject);
    
  /**
   * Apply actuation to the specified cable.
   * @param[in] subject - the RPModel that is being controlled. Must
   * have a list of allMuscles populated
   * @param[in] dt, current timestep must be positive
   */
  virtual void onStep(PrismModel& subject, double dt);
  
  double generateGaussianNoise(double mu, double sigma);
  
private:

  int jetnumber;
  
  double mass;
  
  std::vector<btVector3> force;

  std::vector<btVector3> jetDirections;
  std::vector<btVector3> tankOrientations;

  double goalAltitude, goalYaw;
  double prev_angle_err;
  double error_sum;
  btVector3 goalVector;
  btScalar robotSpeed = 0;

  int activate_flag;
  int transfer_flag;
  int transfer_flag_2;
  
  std::ofstream sim_out;

  //btRigidBody* tankRigidBody;
  //btRigidBody* thrusterRigidBody;
  
};

#endif 
