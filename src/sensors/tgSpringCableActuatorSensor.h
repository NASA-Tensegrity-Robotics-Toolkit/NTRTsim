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

/**
 * @file tgSpringCableActuatorSensor.h
 * @brief Constains definition of concrete class tgSpringCableActuatorSensor.
 * @author Drew Sabelhaus
 * $Id$
 */

#ifndef TG_SPRING_CABLE_ACTUATOR_SENSOR_H
#define TG_SPRING_CABLE_ACTUATOR_SENSOR_H

// Includes from the sensors directory:
#include "tgSensor.h"
// Includes from the NTRT core directory:
#include "core/tgSpringCableActuator.h"

/**
 * This class extends tgSensor to sense a tgSpringCableActuator.
 * Its functionality is similar to what was hard-coded in earlier work
 * on tgDataLogger/Observer.
 */
class tgSpringCableActuatorSensor : public tgSensor
{
public:

  /**
   * The constructor for tgSpringCableActuatorSensor, 
   * will be same as tgSensor, but now with a more specific pointer.
   * @param[in] pSCA a pointer to a tgSpringCableActuator that 
   * this sensor will attach itself to.
   */
  tgSpringCableActuatorSensor(tgSpringCableActuator* pSCA);

  // Classes with virtual member functions must also have virtual destructors.
  virtual ~tgSpringCableActuatorSensor();

  /**
   * Similarly, this class will implement the two data colleciton methods.
   */
  virtual std::vector<std::string> getSensorDataHeadings();
  virtual std::vector<std::string> getSensorData();

};

#endif //TG_SPRING_CABLE_ACTUATOR_SENSOR_H
