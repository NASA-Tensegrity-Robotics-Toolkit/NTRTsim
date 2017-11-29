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

#ifndef LAIKA_WALKING_MODEL_H
#define LAIKA_WALKING_MODEL_H

/**
 * @file LaikaWalkingModel.h
 * @brief Contains the definition of class LaikaWalkingModel
 * @author Drew Sabelhaus
 * $Id$
 */

// This library
#include "yamlbuilder/TensegrityModel.h"
#include "core/tgBasicActuator.h"

// The C++ Standard Library
#include <map>
#include <set>
#include <string>
#include <vector>

// Forward declarations
//class tgWorld; // will we need this for adding the btHingeConstraint?

// This class will inherit from TensegrityModel, and we don't really
// need to do much else besides add some extra methods to get the rigid bodies
// and cables.
class LaikaWalkingModel: public TensegrityModel
{
public:

    /**
     * The two constructors. Will just pass in to the parent
     */
    LaikaWalkingModel(const std::string& structurePath);
    LaikaWalkingModel(const std::string& structurePath, bool debugging);

    /**
     * Nothing to do. Most functions already handled by tgModel::teardown
     */
    virtual ~LaikaWalkingModel()
    {}

    /**
     * Create the model. Call the parent's method,
     * then add a little extra in.
     */
    virtual void setup(tgWorld& world);

    /**
     * This function will return the rigid body states of each of the
     * bodies in the model.
     * Note that it is hand-tuned to only select out specific rigid bodies,
     * and must be modified in the cpp file to return more or less states.
     */
    std::vector<double> getLaikaWalkingModelStates();

    /**
     * This function will return the cable rest lengths.
     * Note that it is hand-tuned to only select out specific rigid bodies,
     * and must be modified in the cpp file to return more or less states.
     */
    std::vector<double> getLaikaWalkingModelCableRL();

		/**
     * Get all cable actuators.
     */
		std::vector<tgBasicActuator*> getAllActuators(std::vector<std::string> actuatorTags);

    std::vector<btRigidBody*> getAllBodies();

private:

  /**
   * Number of vertebrae in the model
   */
  int numVertebrae = 5;

  std::vector<tgBasicActuator*> m_allActuators;
};

#endif
