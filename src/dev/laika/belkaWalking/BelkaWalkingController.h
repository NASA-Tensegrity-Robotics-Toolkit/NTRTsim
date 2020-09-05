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

#ifndef BELKA_WALKING_CONTROLLER_H
#define BELKA_WALKING_CONTROLLER_H

/**
 * @file BelkaWalkingController.h
 * @brief Contains the definition of class BelkaWalkingController.
 * @author Drew Sabelhaus
 * $Id$
 */

// The NTRT core library
#include "core/tgObserver.h"
#include "core/tgSubject.h"
#include "core/tgTags.h"

// The C++ standard library
#include <string>
#include <vector>
#include <map>

#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h" // for hinge hack

// Forward declarations
// class BelkaWalkingModel;
class TensegrityModel;
class tgBasicActuator;

/**
 * A controller for Belka's spine and legs, whole thing.
 */
class BelkaWalkingController : public tgObserver<TensegrityModel>, public tgSubject<BelkaWalkingController>
{
public:
	
  /**
   * Construct a BelkaWalkingController. We're hard-coding the cables so no arguments.
   */
  BelkaWalkingController();
    
  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~BelkaWalkingController() { }

  /**
   * Set up the controller (finding pointers and such)
   * @param[in] subject - the BelkaWalkingModel that is being controlled.
   */
  virtual void onSetup(TensegrityModel& subject);
    
  /**
   * Take the next control step
   * @param[in] subject - the BelkaWalkingModel that is being controlled.
   * @param[in] dt, current timestep must be positive
   */
  virtual void onStep(TensegrityModel& subject, double dt);

protected:

  /**
   * A helper function to find and initialize the actuators that this class
   * will control.
   * @param[in] tag, a string of the tag for which to search in the list of 
   * actuators in this model.
   */
  void initializeActuators(TensegrityModel& subject, std::string tag);
    
private:
	
  /**
   * The private variables for each of the values. Now created in constructor, hard-coded.
   */
  std::vector<std::string> cableTags;

  /**
   * Let's keep our own accumulator... though we really should be asking the simulation
   */
  double m_timePassed;

  /**
   * To keep track of all the actuators, we need a dictionary of lists.
   * Each dictionary key is a tag, and value is the list of pointers to all the cables with those tags.
   */
  std::map<std::string, std::vector<tgBasicActuator*> > cable_ptrs;

  /**
   * The start length of each of the cables must be recorded.
   * The vector here is ordered according to the same list in cable_ptrs.
   */
  std::map<std::string, std::vector<double> > init_rest_lens;

  // From the model, store the leg hinges.
  std::vector<btHingeConstraint*> legHinges;

  // a global constant: max motor impulse. Since we want high stiffness here let's let the motor be powerful. Maybe 2000?
  // This creates a bad large impulse on the ground. Make it smaller so the robot doesn't kick itself upward.
  double max_im = 0.1;

  /**
   * The control inputs here will be the four legs, then spine L/R and CW/CCW. Total of 6.
   * We'll map the L/R and CW/CCW into percentages of each of the relevant tagged cables.
   */
  // NOTE: THIS IS NOW IN BelkaWalkingModel!!!
  // std::vector<double> u_in;

};

#endif // BELKA_WALKING_CONTROLLER_H
