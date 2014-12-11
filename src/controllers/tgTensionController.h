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

#ifndef SRC_CONTROLLERS_TG_TENSION_CONTROLLER_H
#define SRC_CONTROLLERS_TG_TENSION_CONTROLLER_H

/**
 * @file tgTensionController.h
 * @brief Definition of the tgTensionController base class
 * @author Brian Mirletz
 * @date December 2014
 * $Id$
 */

#include "tgBasicController.h"

// Forward declarations
class tgControllable;
class tgBasicActuator;

/**
 * Provides control of rest length based on a tension setpoint for
 * tgBasicActuators.
 */
class tgTensionController : public tgBasicController
{
public:

    /**
	 * The only constructor with two inputs
     * @param[in] controllable. The tgBasicActuator to be controlled.
     * @param[in] setPoint. The initial setPoint for the system
	 */
    tgTensionController(tgBasicActuator* controllable, double setPoint = 0.0);

    /**
     * The destructor. Sets m_controllable to NULL.
     */
    virtual ~tgTensionController();
	
	/**
	 * The control step. This version calculates the necessary rest length
     * change in order to produce the desired tension set point in the actuator
	 * @param[in] dt - the elapsed time since the last call. Must be positive.
	 */
	virtual void control(double dt);
    
    /**
	 * Calls setNewSetPoint on the setPoint parameter, then calls
	 * control(dt)
	 * @param[in] dt - the elapsed time since the last call. Must be positive.
	 * @param[in] setPoint - the setpoint to be used at this step.
	 * @param[in] sensorData: unused in this version. Unifies the API
	 * with PIDController
	 */
	virtual void control(double dt, double setPoint, double sensorData = 0);
	
    /**
     * A static version which directly calls setControlInput(input, dt)
     * on the tgBasicActuator. Calculates correct control input based
     * on the setpoint and properties of the Basic Actuator, performs
     * the same calculation as control(dt)
     * @param[in] sca, a reference the tgBasicActuator to be controlled
     * @param[in] dt, the time elapsed since the last call. Is passed
     * through to tgBasicActuator.moveMotors(dt)
     * @param[in] setPoint, the desired tension.
     */
    static void control(tgBasicActuator& sca, double dt, double setPoint);
private:
    /**
     * The tgBasicActuator this class controls. We do not own this
     * @todo is it better to keep this pointer or cast back from tgControllable?
     */
    tgBasicActuator* m_sca;
	
};

#endif  // SRC_CONTROLLERS_TG_TENSION_CONTROLLER_H

