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
 * @file AATController.cpp
 * @brief Implementation of the deployable heat shield.
 * @author Aliakbar Toghyan
 * @version 1.1.0
 * $Id$
 */

// This module
#include "AATController.h"
// This application
#include "AATModel.h"
// This library
#include "core/tgBasicActuator.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <iostream>
#include <cmath>       

AATController::AATController()
{
	total_time = 0.0;
	m_setPoint = 0.01;
	m_intError = 0.0;
	m_dError = 0.0;
	error = 0.0;
	result = 0.0;
	m_prevError = 0.0;
	KP = 30;
	KI = 10;
	KD = 20;


}

AATController::~AATController()
{
}	

void AATController::onSetup(AATModel& subject)
{
	total_time = 0.0;
}

void AATController::onStep(AATModel& subject, double dt)
{
	total_time+=dt;
	const std::vector<tgBasicActuator*> spring_cables = subject.getAllActuators();
	if (total_time < 0)
	{
		std::cout <<total_time <<"\tCurrent Length " << spring_cables[0]->getCurrentLength() 
	<< "\tTension" << spring_cables[0]->getTension() << "\tRest Length " <<  spring_cables[0]->getRestLength() <<
	"\tTension passive" << spring_cables[8]->getTension() <<std::endl;

	for (unsigned i=0; i<8;++i){
		spring_cables[i+8]->setControlInput(0.01,dt);
		}	
	}
	else
	{
		
	std::cout <<total_time <<"\tCurrent Length " << spring_cables[0]->getCurrentLength() 
	<< "\tTension" << spring_cables[0]->getTension() << "\tRest Length " <<  spring_cables[0]->getRestLength() <<
	"\tTension passive " << spring_cables[8]->getTension() << "\tRest Length passive " << spring_cables[8]->getRestLength() <<std::endl;

	//spring_cables[0]->setControlInput(0.01,dt);
	for (unsigned i=0; i<8;++i){
		
		spring_cables[i+8]->setControlInput(20,dt);
		spring_cables[i]->setControlInput(0.01,dt);
		// for (unsigned i=0; i<8;++i){
	// 	spring_cables[i]->moveMotors(dt);
	// }
	// error = m_setPoint - spring_cables[i]->getCurrentLength();
	
	// /// Integrate using trapezoid rule to reduce error in integration over rectangle
	// m_intError += (error + m_prevError) / 2.0 * dt;
	// m_dError = (error - m_prevError) / dt;
	// result = KP * error + KI * m_intError +
	// 				KD * m_dError;
	
	// spring_cables[i]->setControlInput(result,dt);
	
	// m_prevError = error;
	
	}
	
}
	
	
}
