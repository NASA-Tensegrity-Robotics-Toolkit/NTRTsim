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
 * @file tgSenseable.h
 * @brief Constains the implementation of mixin class tgSenseable.
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
#include "tgSenseable.h"

// From the C++ standard library:
// ...


/**
 * Return an empty vector of descendants.
 * It's up to child classes to re-implement this.
 */
std::vector<tgSenseable*> tgSenseable::getSenseableDescendants() const{
  // For this base class, no descendants are present.
  // In fact, this method should never be called, only the subclasses'
  // methods should be called!
  std::cout << "WARNING! No tgSenseable descendants are present. Are you calling the right function?." << std::endl;
  return std::vector<tgSenseable*>();
}

