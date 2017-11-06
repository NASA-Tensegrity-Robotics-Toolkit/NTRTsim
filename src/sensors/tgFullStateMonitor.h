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

#ifndef TG_FULL_STATE_MONITOR_H
#define TG_FULL_STATE_MONITOR_H

/**
 * @file tgFullStateMonitor.h
 * @brief Contains the definition of class tgFullStateMonitor.
 * @author Drew Sabelhaus
 * $Id$
 */

// Includes from NTRTsim
#include "tgDataManager.h"
// Includes from the C++ standard library
#include <vector> // for storing the states

/**
 * tgFullStateMonitor is a tgDataManager. It records the states from all the 
 * models in the current simulation (reads in a new state at each simulation step.)
 * Then, this class has a method to query the state at any time, independent
 * of onStep or anything from the tgSimulation.
 *
 * Think of this class as the "state" part of state-feedback control.
 * As of 2017-11-05, this class will be designed for only rigid body tgModels:
 * - rods
 * - compound rigids
 * ...all others are ignored (so, in essence, this is a "Full Rigid Body State" monitor.
 */
class tgFullStateMonitor : public tgDataManager
{
 public:

  /**
   * The constructor for tgFullStateMonitor needs no arguments.
   * It will monitor the state in real time (e.g. record a new measurement with
   * every time step.) Maybe if this is slow, we can add in a "sensing period"
   * parameter.
   */
  tgFullStateMonitor();

  /**
   * The base class handles destruction of the sensors and sensorInfos,
   * so nothing to do here.
   */
  ~tgFullStateMonitor();

  /**
   * The setup function for tgFullStateMonitor will:
   * (1) create all the sensors, (2) read an initial measurement from all sensors.
   * Declared virtual here just in case any classes inherit from this.
   */
  virtual void setup();

  /**
   * The teardown function deletes the locally stored arrays and whatnot.
   * TO-DO: should this class also teardown the sensors, or should we let
   * the superclass handle it??
   */
  virtual void teardown();

  /**
   * The step function for tgFullStateMonitor will record the sensor data
   * from each sensor and store it in a local array.
   * @param[in] dt a double, the amount of time since the last step. Not used...
   */
  virtual void step(double dt);

  /**
   * getCurrentState is the externally-facing function that provides access
   * to the currentState vector.
   * Note that we're trying to enforce that users call this method, not the
   * getCurrentStateHelper method, so that the pattern of "monitor records states,
   * then provides its most recent recorded state" holds.
   * That allows for things like e.g. the states will be read at the "proper time"
   * along with everything else, not just whenever your controller gets around
   * to calling getCurrentState.
   */
  // std::vector<double> getCurrentState();

  /**
   * Overwrite toString for the superclass to specify that this data manager
   * is a tgFullStateMonitor.
   */
  virtual std::string toString() const;

  // TO-DO: write a new invariant for this subclass, instead of using the parent's.

 protected:

  /**
   * Store a big vector of all the system states.
   * This should be a vector of doubles, where each are:
   * 12 per rigid body (3 pos, 3 orient, 3 vel, 3 rotational velocity)
   */
  std::vector<double> currentState;

  /**
   * This function queries the sensors and assembles the state vector based
   * the data that is returned. It automatically sets the value of currentState.
   * It should NOT be called outside this class - use getCurrentState for 
   * that instead.
   */
  void getCurrentStateHelper();
  
};


#endif // TG_FULL_STATE_MONITOR_H
