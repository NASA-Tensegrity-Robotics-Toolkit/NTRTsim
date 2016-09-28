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

#ifndef LENGTH_CONTROLLER_H
#define LENGTH_CONTROLLER_H

/**
 * @file RPTensionController.h
 * @brief Contains the definition of class RPTensionController.
 * @author Brian Tietz
 * @version 1.0.0
 * $Id$
 */

// This library
#include "core/tgObserver.h"
#include "controllers/tgBasicController.h"
 #include "core/tgBasicActuator.h"

// The C++ Standard Library
#include <vector>

// Forward declarations
class TensegrityModel;

#include "core/abstractMarker.h" //needed in order to fetch node positions

class LengthController : public tgObserver<TensegrityModel>
{
public:
	
	/**
	 * Construct a LengthTensionController.
	 * @param[in] tension, a double specifying the desired tension
	 * throughougt structure. Must be non-negitive
	 */
    LengthController(const double length = 400);
    
    /**
     * Nothing to delete, destructor must be virtual
     */
    virtual ~LengthController();
    
    virtual void onSetup(TensegrityModel& subject);
    
    /**
     * Apply the length controller. Called by notifyStep(dt) of its
     * subject.
     * @param[in] subject - the RPModel that is being controlled. Must
     * have a list of allMuscles populated
     * @param[in] dt, current timestep must be positive
     */
    virtual void onStep(TensegrityModel& subject, double dt);

    std::vector<tgBasicController*> m_controllers;
    std::vector<double> rand_lengths;
    std::vector<tgSpringCableActuator*> actuators;
    
private:
	
	/**
	 * The tension setpoint that will be passed to the muscles. Set
	 * in the constructor
	 */
    const double m_length;
    double globalTime;
    int toggle;

};

#endif LENGTH_CONTROLLER_H
