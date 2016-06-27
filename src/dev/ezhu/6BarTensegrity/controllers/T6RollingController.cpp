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
 * @file T6RollingController.cpp
 * @brief Implementation of the rolling controller.
 * @author Edward Zhu
 * @version 1.0.0
 * $Id$
 */

// This module
#include "T6RollingController.h"
// The C++ Standard Library
#include <iostream>
#include <cassert>

T6RollingController::Config::Config (const std::string& mode, int face_goal) : 
m_mode(mode), m_face_goal(face_goal)
{
	assert((m_face_goal >= 0) && (m_face_goal <=7));

	if (m_mode.compare("face") != 0) {
		std::cout << "Error: invalid arguments" << std::endl;
		std::cout << "Usage: first arg is a string for mode ('face' or 'dr'). Second arg is based on mode, if 'face' was used, then an int between 0 and 7 is expected. If 'dr' was used, then a btVector3 is expected" << std::endl;
		exit(EXIT_FAILURE);
	}
}

T6RollingController::Config::Config (const std::string& mode, btVector3 dr_goal) :
m_mode(mode), m_dr_goal(dr_goal)
{
	if (mode.compare("dr") != 0) {
		std::cout << "Error: invalid arguments" << std::endl;
		std::cout << "Usage: first arg is a string for mode ('face' or 'dr'). Second arg is based on mode, if 'face' was used, then an int between 0 and 7 is expected. If 'dr' was used, then a btVector3 is expected" << std::endl;
		exit(EXIT_FAILURE);
	}
}

T6RollingController::T6RollingController(const T6RollingController::Config& config) : m_config(config)
{
	c_mode = config.m_mode;
	c_face_goal = config.m_face_goal;
	c_dr_goal = config.m_dr_goal;
}

T6RollingController::~T6RollingController()
{

}

void T6RollingController::onSetup(sixBarModel& subject)
{
	std::cout << c_mode << " mode chosen" << std::endl;
	if (c_mode.compare("face") == 0) {
		std::cout << "Goal face: " << c_face_goal << std::endl;
		controller_mode = 1;
	}
	else {
		std::cout << "Dead reckoning direction: [" << c_dr_goal.x() << ", " 
			<< c_dr_goal.y() << ", " << c_dr_goal.z() << "]" << std::endl;
		controller_mode = 2;
	}

	sixBarRod0 = subject.rodBodies[0];
}

void T6RollingController::onStep(sixBarModel& subject, double dt)
{
	if (dt <= 0.0) {
    	throw std::invalid_argument("dt is not positive");
  	}

  	btVector3 rod0Pos = sixBarRod0->getCenterOfMassPosition();
  	//std::cout << rod0Pos << std::endl;

  	switch (controller_mode) {
  		case 1:
  			// Code for face mode
  			break;
  		case 2:
  			// Code for dead reckoning mode
  			break;
  	}
}