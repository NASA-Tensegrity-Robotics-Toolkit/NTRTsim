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

#ifndef T6_PREFLENGTH_CONTROLLER_H
#define T6_PREFLENGTH_CONTROLLER_H

/**
 * @file T6PrefLengthController.h
 * @brief Contains the definition of class T6PrefLengthController.
 * @author Atil Iscen
 * @version 1.0.0
 * $Id$
 */

// This library
#include "core/tgObserver.h"

// Forward declarations
class T6Model;

/**
 * Preferred Length Controller for T6. This controllers sets a preferred rest length for the muscles.
 * Constant speed motors are used in muscles to move the rest length to the preffered length over time.
 * The assumption here is that motors are constant speed independent of the tension of the muscles.
 * motorspeed and movemotors are defined at the tgLinearString class.
 */
class T6PrefLengthController : public tgObserver<T6Model>
{
public:
	
  /**
   * Construct a T6PrefLengthController with the initial preferred length.
   *
   */
  
  // Note that currently this is calibrated for decimeters.
	T6PrefLengthController(const double prefLength=5);
    
  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~T6PrefLengthController() { }

  virtual void onSetup(T6Model& subject);
    
  virtual void onStep(T6Model& subject, double dt);
    
private:
  double m_initialLengths;
	
};

#endif // T6_PREFLENGTH_CONTROLLER_H
