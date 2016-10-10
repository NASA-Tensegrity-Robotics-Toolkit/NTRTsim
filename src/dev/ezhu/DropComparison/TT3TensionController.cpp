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
 * @file TT3TensionController.cpp
 * @brief Implementation of six strut tensegrity based from Berkeley's TT3 Ball.
 * @author Erik Jung
 * @version 1.1.0
 * $Id$
 */

// This module
#include "TT3TensionController.h"
// This application
#include "TT3Model.h"
// The C++ Standard Library
#include <cassert>
#include <math.h>
#include <stdexcept>

using namespace std;

TT3TensionController::TT3TensionController()
{

}

TT3TensionController::~TT3TensionController()
{
	std::size_t n = m_controllers.size();
	for(std::size_t i = 0; i < n; i++)
	{
		delete m_controllers[i];
	}
	m_controllers.clear();
}

void TT3TensionController::onSetup(TT3Model& subject)
{
	doLog = true;

	if (doLog) {
		std::string filename = "TT3_drop_data.txt";
		// Create filestream for data log and open it
		data_out.open(filename.c_str(), std::fstream::out);
		if (!data_out.is_open()) {
			std::cout << "Failed to open output file" << std::endl;
			exit(EXIT_FAILURE);
		}
		else {
			data_out << "SimTime, XPos, YPos, ZPos, XVel, YVel, ZVel" << std::endl << std::endl;
		}
	}

	std::vector<tgRod*> capsules = subject.getAllCapsules();
	tgRod* capsuleRod = capsules[0];
	capsuleBody = capsuleRod->getPRigidBody();
}

void TT3TensionController::onStep(TT3Model& subject, double dt)
{
	if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else {
    	simTime += dt;
    	std::cout << "Sim time: " << simTime << std::endl;
    }

    if (doLog) {
    	btVector3 capsule_pos = capsuleBody->getCenterOfMassPosition();
    	btVector3 capsule_vel = capsuleBody->getLinearVelocity();
    	data_out << simTime << ", " << capsule_pos.x() << ", " 
    		<< capsule_pos.y() << ", " << capsule_pos.z() << ", " 
    		<< capsule_vel.x() << ", " << capsule_vel.y() << ", " 
    		<< capsule_vel.z() << std::endl;
    }
}