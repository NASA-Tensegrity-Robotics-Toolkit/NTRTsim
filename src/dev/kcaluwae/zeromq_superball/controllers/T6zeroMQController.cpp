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

// This module
#include "T6zeroMQController.h"
// This application
#include "../T6Model.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgKinematicActuator.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <iostream>
#include <cmath>       

T6zeroMQController::T6zeroMQController()
{
	total_time = 0.0;
}

T6zeroMQController::~T6zeroMQController()
{
}	

void T6zeroMQController::onSetup(T6Model& subject)
{
	total_time = 0.0;
	for(unsigned i=0;i<12;++i){
		target_lengths[i] = 0;
	}
}

void T6zeroMQController::setTargetLengths(const float lengths[]){
	for(unsigned i=0;i<12;++i){
		target_lengths[i] = lengths[i];
	}

}

void T6zeroMQController::onStep(T6Model& subject, double dt)
{
	/* Very simple controller that just activates the actuators on the outer circle one by one. */
	total_time+=dt;
	const std::vector<tgBasicActuator*> spring_cables = subject.getAllActuators();
	//std::cout <<total_time <<"\tCurrent Length " << spring_cables[0]->getCurrentLength() << "\tRest Length " << spring_cables[0]->getRestLength()<<std::endl;

	//unsigned motor_idx = int(fmod(total_time,30./2)/(5./2));
	for (unsigned i=0; i<12;++i){
		spring_cables[i]->setControlInput(target_lengths[i]);
	}
	//spring_cables[motor_idx]->setControlInput(6.5);
	for (unsigned i=0; i<12;++i){
		spring_cables[i]->moveMotors(dt);
	}

	
}
