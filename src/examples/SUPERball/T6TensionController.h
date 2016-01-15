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

#ifndef T6_TENSION_CONTROLLER_H
#define T6_TENSION_CONTROLLER_H

/**
 * @file T6TensionController.h
 * @brief Contains the definition of class T6TensionController.
 * @author Brian Tietz
 * @version 1.0.0
 * $Id$
 */

// This library
#include "core/tgObserver.h"
#include "controllers/tgTensionController.h"

// The C++ Standard Library
#include <vector>

// Forward declarations
class T6Model;

/**
 * A controller to apply uniform tension to a T6Model. Iterates through
 * all tgLinearString members and calls tensionMinLengthController
 */
class T6TensionController : public tgObserver<T6Model>
{
public:
	
	/**
	 * Construct a T6TensionController.
	 * @param[in] tension, a double specifying the desired tension
	 * throughougt structure. Must be non-negitive
	 */
    T6TensionController(const double tension = .01);
    
    /**
     * Nothing to delete, destructor must be virtual
     */
    virtual ~T6TensionController();
    
    virtual void onSetup(T6Model& subject);
    
    /**
     * Apply the tension controller. Called my notifyStep(dt) of its
     * subject. The tgLinearStrings will update using
     * their tensionMinLengthController each step
     * @param[in] subject - the T6Model that is being controlled. Must
     * have a list of allMuscles populated
     * @param[in] dt, current timestep must be positive
     */
    virtual void onStep(T6Model& subject, double dt);
    
private:
	
	/**
	 * The tension setpoint that will be passed to the muscles. Set
	 * in the constructor
	 */
    const double m_tension;
    
    std::vector<tgTensionController*> m_controllers;
};

#endif // T6_TENSION_CONTROLLER_H
