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
 * Applies PID control to its tgControllable
 */
class tgPIDController : public tgBasicController
{
public:
	
	struct Config
	{
		Config(double p = 1.0,
				double i = 0.0,
				double d = 0.0,
				double setPoint = 0.0);
		
		const double kP;
		
		const double kI;
		
		const double kD;
		
		const double startingSetPoint;
	};
	
    
    tgPIDController(tgControllable* controllable, tgPIDController::Config config);
    
    
    virtual ~tgPIDController();
	
	virtual void control(double dt);
	
	/**
	 * Calls setSensorData(sensorData), then calls control(dt, setPoint)
	 */
	virtual void control(double dt, double setPoint, double sensorData);

	/**
	 * Need a higher level class to do this which knows more about
	 * the controllable for now
	 * @todo upgrade once the messaging protocol is in place
	 */
	virtual void setSensorData(double sensorData);

private:	
	double m_sensorData;
	
	double m_prevError;
	
	double m_intError;
	
	tgPIDController::Config m_config;
};

#endif  // TG_CONTROLLABLE_H

