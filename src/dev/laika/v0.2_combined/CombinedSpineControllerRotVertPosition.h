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

#ifndef COMBINED_SPINE_CONTROLLER_ROTVERT_POSITION_H
#define COMBINED_SPINE_CONTROLLER_ROTVERT_POSITION_H

/**
 * @file CombinedSpineControllerRotVertPosition.h
 * @brief Contains the definition of class CombinedSpineControllerRotVertPosition.
 * @author Drew Sabelhaus
 * $Id$
 */

// The NTRT core library
#include "core/tgObserver.h"
#include "core/tgSubject.h"
#include "core/tgTags.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
// The C++ standard library
#include <string>
#include <vector>
#include <map>

// Forward declarations
class TensegrityModel;
class tgRod;
class btDynamicsWorld;

/**
 * A controller to track an angular difference (position) between rotating vertebrae 
 * This is used for the IROS 2018 Laika paper results.
 * Note that a specific TensegrityModel, one of the two hinged rods, is what's observed.
 */
class CombinedSpineControllerRotVertPosition : public tgObserver<TensegrityModel>, public tgSubject<CombinedSpineControllerRotVertPosition>
{
public:
	
  /**
   * Construct a CombinedSpineControllerRotVertPosition.
   * Note that the rotation is specified with respect to the local frame
   * @param[in] startTime, a double that determines when the controller
   * begins to track the position.
   * @param[in] setAngle, double, in radians, difference in rotation b/w halves
   * @param[in] rodHingeTag, a string of the tag that's associated with the
   *    tgRods that are part of the hinged joint.
   *@param[in] world, pointer to the btDynamicsWorld that's governing this 
   *    simulation. This is so that the controller can add in the hinge constraint.
   */
  CombinedSpineControllerRotVertPosition(double startTime, double setAngle,
			     std::string rodHingeTag, btDynamicsWorld* world);
    
  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~CombinedSpineControllerRotVertPosition() { }

  /**
   * Create the controller
   * @param[in] subject - the TensegrityModel that is being controlled.
   */
  virtual void onSetup(TensegrityModel& subject);
    
  /**
   * Apply the torque. Note that a torque must be applied at each step,
   * so that it seems like a constant torque is applied in the simulation.
   * @param[in] subject - the TensegrityModel that is being controlled.
   * @param[in] dt, current timestep must be positive
   */
  virtual void onStep(TensegrityModel& subject, double dt);
    
private:
	
  /**
   * The private variables for each of the values passed in to the constructor.
   */
  double m_startTime;
  double m_setAngle;
  std::string m_rodHingeTag;

  // This is a pointer to the first of the two rods with the tag above.
  tgRod* hingedRodA;
  // Second rod:
  tgRod* hingedRodB;

  // Keep track of the world, for adding the hinge constraint.
  btDynamicsWorld* m_world;

  /**
   * Need an accumulator variable to determine when to start the controller.
   */
  double m_timePassed;

  /**
   * Also, need some variables to do PID control in discrete-time.
   */
  double m_accumulatedError; // for I
  double m_prevError; //  for D

};

#endif // COMBINED_SPINE_CONTROLLER_ROTVERT_POSITION_H
