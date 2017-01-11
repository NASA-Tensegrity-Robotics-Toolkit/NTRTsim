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
 * @file tgCompoundRigidSensor.h
 * @brief Constains definition of concrete class tgCompoundRigidSensor.
 * @author Drew Sabelhaus
 * $Id$
 */

#ifndef TG_COMPOUND_RIGID_SENSOR_H
#define TG_COMPOUND_RIGID_SENSOR_H

// Includes from the sensors directory:
#include "tgSensor.h"
// Includes from the NTRT core directory:
#include "core/tgBaseRigid.h"

/**
 * This class extends tgSensor to sense a compound rigid body.
 * It takes in a tgModel as a pointer, as well as a tag of the compound
 * that should be sensed. Note that unlike tgRodSensor etc., the constructor
 * here will necessarily require two arguments: it must answer the question,
 * 'which of the possibly-many compounds within this tgModel should I sense?'
 */
class tgCompoundRigidSensor : public tgSensor
{
public:

  /**
   * The first constructor for tgCompoundRigidSensor.
   * This one takes only a pointer to a tgModel.
   * In the future, it should sense ALL possible compound rigids in the 
   * model, but for now, have it just fail if not provided a tag.
   * @param[in] pModel a pointer to a tgModel that this sensor will attach itself to.
   */
  tgCompoundRigidSensor(tgModel* pModel);

  /**
   * The second constructor for tgCompoundRigidSensor.
   * This one takes a pointer to a tgModel, as well as a single tag
   * that's the compound tag for the compound that should be sensed.
   * @param[in] pModel a pointer to a tgModel that this sensor will attach itself to.
   * @param[in] tag a string that's the compound tag.
   * @throws invalid argument, if the tag doesn't exist in the model.
   */
  tgCompoundRigidSensor(tgModel* pModel, std::string tag);

  // Classes with virtual member functions must also have virtual destructors.
  virtual ~tgCompoundRigidSensor();

  /**
   * Similarly, this class will implement the two data colleciton methods.
   */
  virtual std::vector<std::string> getSensorDataHeadings();
  virtual std::vector<std::string> getSensorData();

};

#endif //TG_COMPOUND_RIGID_SENSOR_H
