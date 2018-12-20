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

#ifndef INV_KIN_TEST_CONTROLLER_H
#define INV_KIN_TEST_CONTROLLER_H

/**
 * @file InvKinTestController.h
 * @brief Contains the definition of class InvKinTestController.
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
// to store references to the cables via a map
class tgBasicActuator;
// For pulling out bodies that will be constrained vertical
class tgTagSearch;

/**
 * A controller to apply a set of inverse kinematics rest lengths as specified by a CSV file.
 */
class InvKinTestController : public tgObserver<TensegrityModel>, public tgSubject<InvKinTestController>
{
public:
	
  /**
   * Construct a InvKinTestController.
   * @param[in] startTime, a start time, after the simulation. Used for the structure to settle into place.
   * @param[in] holdTime, Number of seconds after start time to apply the first input in the CSV file.
   *     This is for the structure to settle into its "starting point" for the control.
   * @param[in] period, the amount of time between switching from one control input to the next. 1/frequency.
   * @param[in] invkinCSVPath the csv file itself, containing rest lengths from the inverse kinematics from MATLAB.
   */
  InvKinTestController(double startTime, double holdTime, double period, std::string invkinCSVPath);
    
  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~InvKinTestController() { }

  /**
   * Create the controller
   * @param[in] subject - the TensegrityModel that is being controlled.
   */
  virtual void onSetup(TensegrityModel& subject);
    
  /**
   * Apply the appropriate rest length to each cable.
   */
  virtual void onStep(TensegrityModel& subject, double dt);
    
private:

  // A function (thanks to stackoverflow) to make the CSV file parsing easier.
  std::vector<std::string> getNextLineAndSplitIntoTokens(std::istream& str);

  /**
   * A series of helper functions to set up the controller.
   * Important: cassignCableInputMap must be called before assignCableTagMap,
   * so that cableInputMap is populated with the tags of the cables to search for.
   */
  void assignCableInputMap();
  void assignCableTagMap(TensegrityModel& subject);
  //void addVerticalConstraint(TensegrityModel& subject, const tgTagSearch& tagSearch);
  void addVerticalConstraint(TensegrityModel& subject, std::string tag);
	
  /**
   * The private variables for each of the values passed in to the constructor.
   */
  double m_startTime;
  double m_holdTime;
  double m_period;
  std::string m_invkinCSVPath;

  /**
   * The private variables assigned by onSetup & helpers.
   * cableTagMap, a map from tag to the corresponding cable (pointer). For ease.
   * cableInputMap, a map from tag to the corresponding vector of rest lengths to apply.
   *  These are read from the CSV. A tag is a string.
   */
  std::map<std::string, tgBasicActuator*> cableTagMap;
  std::map<std::string, std::vector<double> > cableInputMap;

  /**
   * Need an accumulator variable to determine what behavior to do (start, hold, etc.)
   */
  double m_timePassed;

};

#endif // INV_KIN_TEST_CONTROLLER_H
