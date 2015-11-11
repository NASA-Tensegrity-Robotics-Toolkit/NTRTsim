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
 * @file T6TPIDController.cpp
 * @author Ken Caluwaerts
 * @version 1.0.0
 * $Id$
 */

// This module
#include "T6PIDController.h"
// This application
#include "../T6Model.h"
// This library
#include "core/tgKinematicActuator.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <iostream>

T6PIDController::T6PIDController(const ControlMode control_mode,double p,double i,double d) :
    control_mode(control_mode),c_p(p),c_i(i),c_d(d)
{
}

T6PIDController::~T6PIDController()
{
    std::size_t n = m_controllers.size();
    for(std::size_t i = 0; i < n; i++)
    {
        delete m_controllers[i];
    }
    m_controllers.clear();
    m_target.clear();
}	

void T6PIDController::onSetup(T6Model& subject)
{
    std::cout << "onSetup called" << std::endl;
    const std::vector<tgKinematicActuator*> actuators = subject.getAllActuators();
    for (size_t i = 0; i < actuators.size(); ++i)
    {
        tgKinematicActuator * const pActuator = actuators[i];
        assert(pActuator != NULL);
        tgPIDController::Config config(c_p,c_i,c_d);
        tgPIDController* m_PIDController = new tgPIDController(pActuator, config);
        m_controllers.push_back(m_PIDController);
        m_target.push_back(0.);
        prev_rest_length_values.push_back(pActuator->getRestLength());
    }

}

void T6PIDController::onTeardown(T6Model& subject)
{
    std::cout << "onTeardown called" << std::endl;
    std::size_t n = m_controllers.size();
    for(std::size_t i = 0; i < n; i++)
    {
        delete m_controllers[i];
    }
    m_controllers.clear();
    m_target.clear();

}

void T6PIDController::onAttach(T6Model& subject)
{
    std::cout << "onAttach called" << std::endl;
}


void T6PIDController::setTarget(const double target[])
{
    std::size_t n = m_controllers.size();
    for(std::size_t i = 0; i < n; i++)
    {
        m_target[i] = target[i];
    }
}


void T6PIDController::onStep(T6Model& subject, double dt)
{
    assert(dt>0.0);
    std::size_t n = m_controllers.size();
    const std::vector<tgKinematicActuator*> actuators = subject.getAllActuators();

    for(std::size_t i = 0; i < n; i++)
    {
        double m_sensor = 0.;
        tgKinematicActuator * const pActuator = actuators[i];
        switch(control_mode){
            case VELOCITY:
                //DON'T USE VELOCITY CONTROL
                //get current velocity
		//m_sensor = pActuator->getVelocity(); //doesn't work bc tgKinematicActuator is flawed when not backdrivable
                //prev_rest_length_values[i] = pActuator->getRestLength();	
                //break;
            case POSITION:
		//get current spring-cable REST length
                m_sensor = pActuator->getRestLength();
                break;
            case TORQUE:
		m_sensor = 0.; //torque is set directly
                break;
        };
        //if(i==11) 
            //std::cout << i << "\tTarget: " << m_target[i] << "\tSensor: " << m_sensor << "\tVelocity:" << pActuator->getVelocity() << "\tPosition: " << pActuator->getRestLength();
        m_controllers[i]->control(dt, m_target[i], m_sensor);
    }
    //std::cout << std::endl;
}
