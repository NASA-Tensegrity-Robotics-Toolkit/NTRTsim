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

#ifndef TG_BASIC_CONTROLLER_H
#define TG_BASIC_CONTROLLER_H

/**
 * @file tgBasicController.h
 * @brief Definition of the tgBasicController base class
 * @author Brian Mirletz
 * @date December 2014
 * $Id$
 */

// Forward declarations
class tgControllable;

/**
 * The simplest possible controller. Naively passes its setpoint
 * straight through as the control input. Functional for rest length
 * control for a tgBasicActuator.
 */
class tgBasicController
{
public:

    /**
	 * The only constructor with two inputs
     * @param[in] controllable. The system to be controlled.
     * @param[in] setPoint. The initial setPoint for the system
	 */
    tgBasicController(tgControllable* controllable, double setPoint = 0.0);

    /**
     * The destructor. Sets m_controllable to NULL.
     */
    virtual ~tgBasicController();
	
	/**
	 * The control step. In this version it passes the setPoint directly
	 * to m_controllable
	 * @param[in] dt - the timestep. Must be positive.
	 */
	virtual void control(double dt);
	
	/**
	 * Calls setNewSetPoint on the setPoint parameter, then calls
	 * control(dt)
	 * @param[in] dt - the elapsed time. Must be positive.
	 * @param[in] setPoint - the setpoint to be used at this step.
	 * @param[in] sensorData: unused in this version. Unifies the API
	 * with PIDController
	 */
	virtual void control(double dt, double setPoint, double sensorData = 0);
	
    /**
     * Updates m_setPoint to newSetpoint
     * @param[in] newSetPoint, the next desired setpoint for the controllable
     */
	virtual void setNewSetPoint(double newSetPoint);
	
	/**
	 * Return a const pointer to a const tgControllable.
	 * Current solution to obtaining sensor data (requires casting
	 * at a higher level)
	 */
	const tgControllable* const getControllable() const
	{
		return m_controllable;
	}
	
protected:
	
	/**
	 * The control setpoint. Updated via control or setNewSetPoint
	 */
	double m_setPoint;
	
	/**
	 * The system being controlled. We don't own this
	 */
	tgControllable* m_controllable;
	
};

#endif  // TG_BASIC_CONTROLLER_H

