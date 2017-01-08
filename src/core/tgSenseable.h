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
 * @brief Constains definition of mixin class tgSenseable, for objects that can be sensed.
 * @author Drew Sabelhaus
 * $Id$
 */

#ifndef TG_SENSEABLE_H
#define TG_SENSEABLE_H

// From the C++ standard library:
#include <iostream> //for strings
#include <vector> //for lists of descendants

/**
 * This class is used to abstract away the class of pointers
 * that can be passed around in the tgSensors infrastructure.
 * For now, it's really only tgModel that will inherit from it.
 */
class tgSenseable
{
 public:

  /**
   * In order to create sensors for a whole hierarchy of senseable objects,
   * sense-able objects need to have references to thei children.
   * This will be used in a tgDataManager's setup and step functions.
   * For now, this 'should' be handled by tgModel's getDescendants function.
   */
  virtual std::vector<tgSenseable*> getSenseableDescendants() const;
    
};

#endif //TG_SENSEABLE_H
