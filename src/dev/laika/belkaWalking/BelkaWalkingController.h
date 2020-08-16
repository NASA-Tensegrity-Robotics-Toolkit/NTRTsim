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
class BelkaWalkingModel; // not needed, we're directly including the BelkaWalkingModel.h now
class tgBasicActuator;

/**
 * A controller for Belka's spine and legs, whole thing.
 */
class BelkaWalkingController : public tgObserver<BelkaWalkingModel>, public tgSubject<BelkaWalkingController>
{
public:
	
  /**
   * Construct a BelkaWalkingController.
   * @param[in] spineTags, a vector (array) of strings, which is a list of the 
   * tags of all the spine cables to act on (ordered by index.)
   * @param[in] legHingeTags, same as spineTags but for the leg hinges (angular displacement.)
   */
  BelkaWalkingController(std::vector<std::string> spineTags);
    
  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~BelkaWalkingController() { }

  /**
   * Set up the controller (finding pointers and such)
   * @param[in] subject - the BelkaWalkingModel that is being controlled.
   */
  virtual void onSetup(BelkaWalkingModel& subject);
    
  /**
   * Take the next control step
   * @param[in] subject - the BelkaWalkingModel that is being controlled.
   * @param[in] dt, current timestep must be positive
   */
  virtual void onStep(BelkaWalkingModel& subject, double dt);

protected:

  /**
   * A helper function to find and initialize the actuators that this class
   * will control.
   * @param[in] tag, a string of the tag for which to search in the list of 
   * actuators in this model.
   */
  void initializeActuators(BelkaWalkingModel& subject, std::string tag);
    
private:
	
  /**
   * The private variables for each of the values passed in to the constructor.
   */
  std::vector<std::string> m_spineTags;

  /**
   * Let's keep our own accumulator... though we really should be asking the simulation
   */
  double m_timePassed;

  /**
   * The start length of each of the cables must be recorded.
   * This map takes a string (the space-separated list of all the tags for
   * an individual cable) and outputs a double (the rest length at time t=0.)
   */
  // typedef std::map<tgTags, double> InitialRestLengths;
  // InitialRestLengths initialRL;

  /**
   * A list of all the actuators to control. This is populated in onSetup
   * by using m_tagsToControl.
   */
  std::vector<tgBasicActuator*> cablesWithTags;

  // From the model, store the leg hinges.
  std::vector<btHingeConstraint*> legHinges;

};

#endif // BELKA_WALKING_CONTROLLER_H
