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
 * @file tgSphereSensor.h
 * @brief Constains definition of concrete class tgSphereSensor.
 * @author Drew Sabelhaus
 * $Id$
 */

#ifndef TG_SPHERE_SENSOR_H
#define TG_SPHERE_SENSOR_H

// Includes from the sensors directory:
#include "tgSensor.h"
// Includes from the NTRT core directory:
#include "core/tgSphere.h"

/**
 * This class extends tgSensor to sense a tgSphere.
 * Its functionality is similar to what was hard-coded in earlier work
 * on tgDataLogger/Observer.
 */
class tgSphereSensor : public tgSensor
{
public:

  /**
   * The constructor for tgSphereSensor, will be same as tgSensor, but now
   * with a more specific pointer. This should work, since tgSphere
   * is a tgSenseable.
   * @param[in] pSphere a pointer to a tgSphere that this sensor will attach itself to.
   */
  tgSphereSensor(tgSphere* pSphere);

  // Classes with virtual member functions must also have virtual destructors.
  virtual ~tgSphereSensor();

  /**
   * Similarly, this class will implement the two data colleciton methods.
   */
  virtual std::vector<std::string> getSensorDataHeadings();
  virtual std::vector<std::string> getSensorData();

};

#endif //TG_SPHERE_SENSOR_H
