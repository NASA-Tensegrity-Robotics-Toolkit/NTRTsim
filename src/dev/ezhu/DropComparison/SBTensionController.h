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

#ifndef SB_TENSION_CONTROLLER_H
#define SB_TENSION_CONTROLLER_H

/**
 * @file SBTensionController.h
 * @brief Contains the definition of class SBTensionController.
 * @author Erik Jung
 * @version 1.0.0
 * $Id$
 */

// This library
#include "core/tgObserver.h"
#include "core/tgRod.h"
#include "controllers/tgTensionController.h"
#include "core/tgBasicActuator.h"
#include "core/abstractMarker.h"
// The C++ Standard Library
#include <vector>
#include <string>
#include <fstream>
#include <ostream>
//Bullet Physics
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
 //namespace std for vectors 
using namespace std;

// Forward declarations
class SBModel;

/**
 * A controller to apply uniform tension to a SBModel. Iterates through
 * all tgLinearString members and calls tensionMinLengthController
 */
class SBTensionController : public tgObserver<SBModel>
{
public:
	
    SBTensionController();
    /**
     * Nothing to delete, destructor must be virtual
     */
    virtual ~SBTensionController();
    
    virtual void onSetup(SBModel& subject);
    
    /**
     * Apply the tension controller. Called my notifyStep(dt) of its
     * subject. The tgLinearStrings will update using
     * their tensionMinLengthController each step
     * @param[in] subject - the SBModel that is being controlled. Must
     * have a list of allMuscles populated
     * @param[in] dt, current timestep must be positive
     */
    virtual void onStep(SBModel& subject, double dt);
    
private:

    btRigidBody* capsuleBody;

    std::vector<tgTensionController*> m_controllers;

    bool doLog = false;

    std::ofstream data_out;
    double simTime = 0;
    
    abstractMarker marker;
};

#endif // SB_TENSION_CONTROLLER_H
