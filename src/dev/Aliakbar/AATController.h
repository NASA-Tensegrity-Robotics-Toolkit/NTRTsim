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
 * @file AATController.h
 * @brief Contains the definition of class AATController.
 * @author Aliakbar Toghyan
 * @version 1.1.0
 * $Id$
 */


#ifndef AAT_CONTROLLER_H
#define AAT_CONTROLLER_H


// This library
#include "core/tgObserver.h"
#include "controllers/tgTensionController.h"

// The C++ Standard Library
#include <vector>

// Forward declarations
class AATModel;

class AATController : public tgObserver<AATModel>
{
public:
	
	/**
	 * Construct a AATController.
	 */
    AATController();
    
    /**
     * Nothing to delete, destructor must be virtual
     */
    virtual ~AATController();
    
    virtual void onSetup(AATModel& subject);
    
    /**
     * Apply the tension controller. Called my notifyStep(dt) of its
     * subject. The tgLinearStrings will update using
     * their tensionMinLengthController each step
     * @param[in] subject - the AATModel that is being controlled. Must
     * have a list of allMuscles populated
     * @param[in] dt, current timestep must be positive
     */
    virtual void onStep(AATModel& subject, double dt);
    
private:

protected:
	double total_time;

    double m_setPoint;	

    double m_intError;

    double m_dError; 
    
    double error; 

    double m_prevError;

    double result;

    double KP;

    double KI;

    double KD;
};

#endif // AAT_CONTROLLER_H