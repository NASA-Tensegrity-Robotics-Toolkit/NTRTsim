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

#ifndef BELKA_WALKING_FILE_CONTROLLER_H
#define BELKA_WALKING_FILE_CONTROLLER_H

/**
 * @file BelkaWalkingFileController.h
 * @brief Contains the definition of class BelkaWalkingFileController.
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
 * A controller for Belka's spine and legs, whole thing. Now reads inputs from a file.
 */
class BelkaWalkingFileController : public tgObserver<TensegrityModel>, public tgSubject<BelkaWalkingFileController>
{
public:
	
  /**
   * Construct a BelkaWalkingFileController. Now takes one argument, a filename:
   * @param input_traj_path path to the .csv file with the timepoint trajectory of inputs
   */
  BelkaWalkingFileController(std::string input_traj_path);
    
  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~BelkaWalkingFileController() { }

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
  // Note: 0.05 is the minimum that lets the robot right itself. 
  double max_im = 1.0;

  // We'll do a PID feedback controller from angle->velocity on the hinge motor. 
  // With the timestep of 0.00005, the gain corresponding to setMotorTarget's (1/dt) is 20,000
  double m_KP = 5.0;
  double m_KI = 0.001;
  double m_KD = 0.001;
  // Seems we might actually not need much of the I or D at all. Previously, we were effectively doing bang-bang, 
  // with an extremely high P constant and a limit via max_im. Now with true P control it actually seems to be fine
  // double m_KI = 0.05;
  // double m_KD = 500.0;
  // also need some extra states for the I and D feedback. We'll have these for each leg
  // double m_accumulatedError; // for I
  std::vector<double> accum_err;
  // double m_prevError; //  for D.
  std::vector<double> prev_err;

  // Save the path to the csv file so that it can be reused across teardowns
  std::string m_input_traj_path;

  // helper to parse the csv file. Writes data into time_pts and dutys.
  void parseActuationFile(std::string csv_path);

  // Store the timepoints for the control input trajectory. We'll assume index alignment between these two
  std::vector<double> time_pts;
  std::vector<std::vector<double> > input_traj; // at index (whatever), a list of the values for each input.

  // The CSV control trajectory file will have a certain number of lines as a header. We'll need to remove them.
  int csv_header_lines = 4;

  // Used for checking the CSV file: we have only 6 inputs here
  int num_inputs = 6;

};

#endif // BELKA_WALKING_FILE_CONTROLLER_H
