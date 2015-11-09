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

#ifndef v4_TENSION_CONTROLLER_H
#define v4_TENSION_CONTROLLER_H

/**
 * @file v4TensionController.h
 * @brief Contains the definition of class v4TensionController.
 * @author Erik Jung
 * @version 1.0.0
 * $Id$
 */

// This library
#include "core/tgObserver.h"
#include "controllers/tgTensionController.h"
#include "learning/Adapters/AnnealAdapter.h"
#include "core/tgBasicActuator.h"
// The C++ Standard Library
#include <vector>
//Bullet Physics
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
 //namespace std for vectors 
using namespace std;

// Forward declarations
class v4Model;

/**
 * A controller to apply uniform tension to a v4Model. Iterates through
 * all tgLinearString members and calls tensionMinLengthController
 */
class v4TensionController : public tgObserver<v4Model>
{
public:
	
    v4TensionController(const double tension, double timestep, btVector3 goalTrajectory);
    /**
     * Nothing to delete, destructor must be virtual
     */
    virtual ~v4TensionController();
    
    virtual void onSetup(v4Model& subject);
    
    /**
     * Apply the tension controller. Called my notifyStep(dt) of its
     * subject. The tgLinearStrings will update using
     * their tensionMinLengthController each step
     * @param[in] subject - the v4Model that is being controlled. Must
     * have a list of allMuscles populated
     * @param[in] dt, current timestep must be positive
     */
    virtual void onStep(v4Model& subject, double dt);
    
private:
	
	/**
	 * The tension setpoint that will be passed to the muscles. Set
	 * in the constructor
	 */
    //const double m_tension;
    double m_totalTime;
    double m_initialLengths;
    btVector3 initPos;
    btVector3 trajectory;
    //btVector3 goal;

    std::vector<tgTensionController*> m_controllers;

    btVector3 endEffectorCOM(v4Model& subject);
    

};

#endif // v4_TENSION_CONTROLLER_H
