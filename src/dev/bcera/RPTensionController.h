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

#ifndef RP_TENSION_CONTROLLER_H
#define RP_TENSION_CONTROLLER_H

/**
 * @file RPTensionController.h
 * @brief Contains the definition of class RPTensionController.
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
class RPModel;

/**
 * A controller to apply uniform tension to a RPModel. Iterates through
 * all tgLinearString members and calls tensionMinLengthController
 */
class RPTensionController : public tgObserver<RPModel>
{
public:
	
	/**
	 * Construct a RPTensionController.
	 * @param[in] tension, a double specifying the desired tension
	 * throughougt structure. Must be non-negitive
	 */
    RPTensionController(const double tension = 400);
    
    /**
     * Nothing to delete, destructor must be virtual
     */
    virtual ~RPTensionController();
    
    virtual void onSetup(RPModel& subject);
    
    /**
     * Apply the tension controller. Called my notifyStep(dt) of its
     * subject. The tgLinearStrings will update using
     * their tensionMinLengthController each step
     * @param[in] subject - the RPModel that is being controlled. Must
     * have a list of allMuscles populated
     * @param[in] dt, current timestep must be positive
     */

    virtual int findClosestNode(RPModel& subject);

    virtual void onStep(RPModel& subject, double dt);

    std::vector<tgTensionController*> m_controllers;
    std::vector<tgBasicController*> m_controllers_length;
    
private:
	
	/**
	 * The tension setpoint that will be passed to the muscles. Set
	 * in the constructor
	 */
    const double m_tension;
    double globalTime,sec_count;
    int toggle;

    

};

#endif // RP_TENSION_CONTROLLER_H
