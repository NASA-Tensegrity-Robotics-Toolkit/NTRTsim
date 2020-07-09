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
 * @file tgSensor.cpp
 * @brief An implementation of some parts of the tgSensor abstract class.
 * @author Drew Sabelhaus
 * @date January 4, 2017
 * $Id$
 */

// Includes from NTRT:
#include "tgSensor.h"
#include "core/tgSenseable.h"

// Includes from the c++ standard library:
#include <stdexcept>

/**
 * This cpp file only implements the constructor for tgSensor.
 * Note that tgSensor is an abstract class with two pure virtual member
 * functions, so you cannot instantiate a tgSensor.
 * However, a constructor is provided here for ease of managing pointers
 * in child classes.
 * The shorthand syntax for variable assignment is used here.
 * "m_pSens" stands for "my pointer to a tgSenseable object."
 */
tgSensor::tgSensor(tgSenseable* pSens) : m_pSens(pSens)
{
  if (pSens == NULL) {
    throw std::invalid_argument("Pointer to pSenseable is NULL inside tgSensor.");
  }
}

/** A class with virtual member functions must have a virtual destructor. */
tgSensor::~tgSensor()
{
  // Note that we should NOT be deleting the m_pSens, since that's
  // likely a tgModel, which is handled by other classes.
}

//end.
