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

#ifndef TG_WORLDIMPL_H
#define TG_WORLDIMPL_H

/**
 * @file tgWorldImpl.h
 * @brief Contains the definition of class tgWorldImpl
 * @author Lee Brownston
 * $Id$
 */

// Solves a compiler error. See if we can make it a forward declaration again
#include "tgWorld.h"

// Forward declarations
class tgGround;

/**
 * Abstract base class to encapsulate the implementation of the tgWorld.
 * The tgWorld owns an instance of this.
 */
class tgWorldImpl
{
public:
  
  /** 
   * The only constructor. The base class initializes nothing.
   * @param[in] config configuration POD
   */
  tgWorldImpl(const tgWorld::Config& config, const tgGround* const ground) { }
   
  /** Clean up the implementation. The base class holds nothing. */
  virtual ~tgWorldImpl() { }

  /**
   * Advance the simulation.
   * @param[in] dt the number of seconds since the previous call;
   * must be positive
   */
  virtual void step(double dt) = 0;
};


#endif  // TG_WORLDIMPL_H
