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

//#define PI 3.14159265

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

// Forward declarations
class RPModel;

/**
 * A controller that applies thrust to a center payload.
 */
class RPThruster : public tgObserver<RPModel>
{
public:
	
  /**
   * Construct a RPThruster.
   * @param[in] thrust, specifies magnitude of thrust to be applied.
   */
  
  // Note that currently this is calibrated for decimeters.
  RPThruster(const int thrust = 0);
    
  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~RPThruster() { }

  /**
   * On setup, this controller does nothing.
   * @param[in] subject - the RPModel that is being controlled. Must
   * have a list of allMuscles populated
   */
  virtual void onSetup(RPModel& subject);
    
  /**
   * Apply actuation to the specified cable.
   * @param[in] subject - the RPModel that is being controlled. Must
   * have a list of allMuscles populated
   * @param[in] dt, current timestep must be positive
   */
  virtual void onStep(RPModel& subject, double dt);
  
  /**
   * Kyunam's custom functions
   */    
  void saveOldThrust(double thrust);
  
  void saveOldPhi(double phi);
  
  void saveOldTheta(double theta);
  
  double getOldThrust();
  
  double getOldPhi();
  
  double getOldTheta();

  double computeNextTargetTheta(btVector3 currentLocation, btVector3 targetLocation);
  
  double generateGaussianNoise(double mu, double sigma);
  
private:
	
  /**
   * The thrust to be applied. Set in the constructor.
   */
  const int m_thrust;

  int jetnumber = 7;
  
  double mass;
  
  double oldThrust = 50.0; // set initial thrust 
  
  double oldPhi = 90.0*M_PI/180.0; // set initial phi = 45 deg

  double oldTheta = 0.0; // set initial theta = 0 deg s.t. robot head towards +X direction
  
  std::vector<btVector3> force;

  std::vector<btVector3> jetDirections;
  
};

#endif 
