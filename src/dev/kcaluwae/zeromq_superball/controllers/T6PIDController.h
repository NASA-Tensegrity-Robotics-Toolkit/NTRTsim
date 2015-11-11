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

#ifndef T6_PID_CONTROLLER_H
#define T6_PID_CONTROLLER_H

/**
 * @file T6PIDController.h
 * @brief Contains the definition of class T6PIDController.
 * @author Ken Caluwaerts
 * @version 1.0.0
 * $Id$
 */

// This library
#include "core/tgObserver.h"
#include "controllers/tgPIDController.h"

// The C++ Standard Library
#include <vector>

// Forward declarations
class T6Model;

/**
 * A PID controller that controls the torque of a set of tgKinematicActuators 
 * in order to track position, velocity or (trivially) torque.
 */
class T6PIDController : public tgObserver<T6Model>
{
public:
    enum ControlMode {TORQUE, VELOCITY, POSITION};
	
	/**
	 * Construct a T6PIDController.
	 * @param[in] tension, a double specifying the desired tension
	 * throughougt structure. Must be non-negitive
	 */
    T6PIDController(const ControlMode control_mode = POSITION,double p=1.,double i=0.,double d=0.);
    
    /**
     * Nothing to delete, destructor must be virtual
     */
    virtual ~T6PIDController();
    
    virtual void onSetup(T6Model& subject);
    virtual void onTeardown(T6Model& subject);
    virtual void onAttach(T6Model& subject);
    
    /**
     * Apply the PID controller. 
     * @param[in] subject - the T6Model that is being controlled. Must
     * have a list of allMuscles populated
     * @param[in] dt, current timestep must be positive
     */
    virtual void onStep(T6Model& subject, double dt);

    /**
    * Sets the target values (velocity, position or torque) of the actuators.
    *
    */
    virtual void setTarget(const double target[]);
    
private:

    /** Target velocity, torque or position depending on mode for each actuator */	
    std::vector<double> m_target; 
    
    std::vector<tgPIDController*> m_controllers;
    std::vector<double> prev_rest_length_values;

    ControlMode control_mode;

    /** PID parameters */
    double c_p, c_i, c_d;
};

#endif // T6_PID_CONTROLLER_H
