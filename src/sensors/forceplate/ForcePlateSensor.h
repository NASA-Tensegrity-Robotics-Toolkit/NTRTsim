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

#ifndef FORCE_PLATE_SENSOR_H
#define FORCE_PLATE_SENSOR_H

/**
 * @file ForcePlateSensor.h
 * @brief Contains the definition of class ForcePlateSensor.
 * @author Drew Sabelhaus
 * @version 1.0.0
 * $Id$
 */

// This library
#include "core/tgObserver.h"
#include "core/tgSubject.h"

// the data collection class
#include "sensors/tgDataObserver.h"
#include <string>

// Forward declarations
class ForcePlateModel;

/**
 * This class provides a sensor that can be attached to a Force Plate and which
 * will take data from the force plate.
 * See examples/forcePlateDemo for an example.
 * Note that it is similar to the "controller" types of files that NTRT uses.
 * since it also tgObserver, but does not control or actuate the force plate.
 * Similar to those controllers, it also needs to be tgSubject, itself, so that
 * the data logging infrastructure will work.
 */
class ForcePlateSensor : public tgObserver<ForcePlateModel>, public tgSubject <ForcePlateSensor>
{
public:
	
  /**
   * Construct a ForcePlateSensor.
   * @param[in] path, a string that represents the path to the folder where the 
   * log file will be created when this class is constructed and run.
   * Note that the simulation will crash if the directory of this directory
   * does not exist.
   * @param[in] timeBetweenSamples, the length of time in between readings of the
   * force from this force plate. If set very small, then a massive large log file
   * will result, and you may encounter trouble reading that file into e.g. MATLAB.
   */
  
  ForcePlateSensor(std::string path, double timeBetweenSamples);
    
  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~ForcePlateSensor() { }

  /**
   * The setup method here just calls the setup method
   * for the data observer. This will open the log file for writing.
   * @param[in] subject - the ForcePlateModel that is being observed.
   */
  virtual void onSetup(ForcePlateModel& subject);
    
  /**
   * The onStep method calls the data observer to record data about
   * the force plate at a particular timestep.
   * @param[in] subject - the ForcePlateModel
   * @param[in] dt, current timestep must be positive
   */
  virtual void onStep(ForcePlateModel& subject, double dt);
    
private:

  /**
   * Stored from the constructor: the amount of time to wait
   * in between samples. Setting this too low will ouput a huge log file.
   */
  double m_timeBetweenSamples;
  
  /**
   * A counter variable that increments at each call of onStep.
   * By incrementing updateTime, it's possible to check when to 
   * sample the force plate (when updateTime > m_timeBetweenSamples.)
   */
   double m_updateTime;

   /**
    * This class owns a data observer. That data observer then calls
    * the data logger to output the actual log file.
    */
  tgDataObserver m_dataObserver;

};

#endif // FORCE_PLATE_SENSOR_H
