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

#ifndef TG_PID_CONTROLLER_H
#define TG_PID_CONTROLLER_H

/**
 * @file tgPIDController.h
 * @brief Definition of the tgPIDController class
 * @author Brian Mirletz
 * @date December 2014
 * $Id$
 */

#include "tgBasicController.h"

// Forward declarations
class tgControllable;

/**
 * Applies PID control to its tgControllable. Will work for any controllable
 * for which we can get an approprate sensor input. Depending on the system
 * it may be necessary to invert the output, this is done within the 
 * config file upon construction.
 */
class tgPIDController : public tgBasicController
{
public:
	
	struct Config
	{
		/**
		 * The config's constructor
		 * @param[in] p, the position gain
		 * @param[in] i, the velocity gain
		 * @param[in] d, the derivative gain
         * @param[in] tensControl, whether or not this PID controller will
         * be used in tension control, 
		 * @param[in] setPoint, the initial setpoint
		 */
		Config(double p = 1.0,
				double i = 0.0,
				double d = 0.0,
                bool tensControl = false,
				double setPoint = 0.0);
		
		/**
		 * The position gain of the PID controller. Initial value must
         * be nonnegative
		 * Use a negative gain for tension control! (set tensControl to true)
		 * Units are applicaiton dependant 
		 */
		const double kP;
		
		/**
		 * The integral gain of the PID controller. Initial value must
         * be nonnegative
		 * Use a negative gain for tension control! (set tensControl to true)
		 * Units are applicaiton dependant 
		 */
		const double kI;
		
		/**
		 * The derivative gain of the PID controller. Initial value must
         * be nonnegative
		 * Use a negative gain for tension control! (set tensControl to true)
		 * Units are applicaiton dependant 
		 */
		const double kD;
		
		/**
		 * The initial setpoint for the controller. Sets the value
		 * for m_setPoint
		 */
		const double startingSetPoint;
	};
	
    /**
     * The only constructor. 
     * @param[in] controllable. The system to be controlled. One to
     * one mapping with PID controller, since history is kept in the 
     * controller through the previous error and integral parameters
     * @param[in] config. Contains the gains for the PID control and 
     * a starting setpoint
     */
    tgPIDController(tgControllable* controllable, tgPIDController::Config config);
    
	/**
	 * This destructor has nothing to do. The parent class will set
	 * the pointer for m_controllable to NULL
	 */
    virtual ~tgPIDController();
	
	/**
	 * Calls the main PID loop. Updates m_prevError and m_intError
	 * and calls setControlInput on m_controllable
	 * @param[in] dt - the timestep. Must be positive.
	 * @todo determine whether this should be public
	 */
	virtual void control(double dt);
	
	/**
	 * The standard public call for running the PID loop and applying
	 * the output to the controlled system
	 * Calls setSensorData(sensorData), then sets m_setPoint equal to
	 * setpoint and calls control(dt)
	 * @param[in] dt - the timestep. Must be positive.
	 * @param[in] setPoint - the setpoint to be used at this step.
	 * @param[in] the value with which the setpoint will be compared
	 */
	virtual void control(double dt, double setPoint, double sensorData);
    
	/**
	 * Need a higher level class to do this which knows more about
	 * the controllable for now
	 * @todo upgrade once the messaging protocol is in place
	 */
	virtual void setSensorData(double sensorData);
	
	/// @todo should we have a getSensorData function? Might make code changes simpler later
	
private:
	/**
	 * Member variable for sensor data. Units are application dependant.
	 */
	double m_sensorData;
	
	/**
	 * The previous error between the setpoint and the sensor value.
	 * Used in computing the integral and derivative terms
	 */
	double m_prevError;
	
	/**
	 * The integral of the error
	 */
	double m_intError;
	
	/**
	 * A locally stored config, contains the gains
	 */
	const tgPIDController::Config m_config;
};

#endif  // TG_PID_CONTROLLER_H

