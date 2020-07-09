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
#include "core/tgModel.h"
#include "core/tgBaseRigid.h"
// Includes from the C++ standard library:
#include <vector>
// Includes from Bullet Physics:
#include "LinearMath/btVector3.h" //for the functions calculating com and orient
#include "LinearMath/btQuaternion.h" // for the initial orientation variable

/**
 * This class extends tgSensor to sense a compound rigid body.
 * It takes in a tgModel as a pointer.
 * Note that unlike tgRodSensor etc., a tgCompoundRigidSensor
 * requires that a tag of the compound to sense is passed in.
 * In order for this sensor framework to function "nicely", one 
 * tgCompoundRigidSensor should only output data from one compound rigid.
 * That requires specifying WHICH compound rigid to sense, since many
 * tgModels have more than one compound rigid inside them.
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
   * @throws invalid_argument since this constructor should never be called, 
   * it's always an invalid argument.
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
   * Note that for getSensorDataHeadings, the only tag included here is 
   * the compound rigid body tag. TO-DO: maybe take the union of all tags
   * of the consistutent rigid bodies?
   */
  virtual std::vector<std::string> getSensorDataHeadings();
  virtual std::vector<std::string> getSensorData();

 private:

  /**
   * Three methods that calculate the sensor information from the pool of
   * rigid bodies.
   */
  btVector3 getCenterOfMass();
  btVector3 getOrientation();
  double getMass();

  /**
   * This sensor keeps track of the compound tag that it will be sensing.
   */
  std::string m_tag;
  
  /**
   * A compound rigid sensor has a list of rigid bodies
   * that are together in its compound.
   */
  std::vector<tgBaseRigid*> m_rigids;

  /**
   * Store the original orientation of the compound rigid,
   * for comparison later to get the current orientation.
   * We do this by selecting the first rigid body from m_rigids.
   * Also, store the inverse: we'll be using the inverse at each step,
   * better to calculate it now.
   * TO-DO: that means we need to make sure that list doesn't change!
   */
  btQuaternion origOrientQuat;
  btQuaternion origOrientQuatInv;

};

#endif //TG_COMPOUND_RIGID_SENSOR_H
