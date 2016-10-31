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
			// data_out << "SimTime, " <<
			// "R0XPos, R0YPos, R0ZPos, R0Ang, R0AxX, R0AxY, R0AxZ, " <<
			// "R1XPos, R1YPos, R1ZPos, R1Ang, R1AxX, R1AxY, R1AxZ, " <<
			// "R2XPos, R2YPos, R2ZPos, R2Ang, R2AxX, R2AxY, R2AxZ, " <<
			// "R3XPos, R3YPos, R3ZPos, R3Ang, R3AxX, R3AxY, R3AxZ, " <<
			// "R4XPos, R4YPos, R4ZPos, R4Ang, R4AxX, R4AxY, R4AxZ, " <<
			// "R5XPos, R5YPos, R5ZPos, R5Ang, R5AxX, R5AxY, R5AxZ" << std::endl << std::endl;

            data_out << "SimTime, " <<
            "R0XPos, R0YPos, R0ZPos, R0Yaw, R0Pitch, R0Roll, " <<
            "R1XPos, R1YPos, R1ZPos, R1Yaw, R1Pitch, R1Roll, " <<
            "R2XPos, R2YPos, R2ZPos, R2Yaw, R2Pitch, R2Roll, " <<
            "R3XPos, R3YPos, R3ZPos, R3Yaw, R3Pitch, R3Roll, " <<
            "R4XPos, R4YPos, R4ZPos, R4Yaw, R4Pitch, R4Roll, " <<
            "R5XPos, R5YPos, R5ZPos, R5Yaw, R5Pitch, R5Roll" << std::endl << std::endl;
		}
	}

	std::vector<tgRod*> rods = subject.getAllRods();
	tgRod* Rod0 = rods[0];
	rodBody0 = Rod0->getPRigidBody();
	tgRod* Rod1 = rods[1];
	rodBody1 = Rod1->getPRigidBody();
	tgRod* Rod2 = rods[2];
	rodBody2 = Rod2->getPRigidBody();
	tgRod* Rod3 = rods[3];
	rodBody3 = Rod3->getPRigidBody();
	tgRod* Rod4 = rods[4];
	rodBody4 = Rod4->getPRigidBody();
	tgRod* Rod5 = rods[5];
	rodBody5 = Rod5->getPRigidBody();
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

    btVector3 rod_pos0 = rodBody0->getCenterOfMassPosition();
    btVector3 rod_pos1 = rodBody1->getCenterOfMassPosition();
    btVector3 rod_pos2 = rodBody2->getCenterOfMassPosition();
    btVector3 rod_pos3 = rodBody3->getCenterOfMassPosition();
    btVector3 rod_pos4 = rodBody4->getCenterOfMassPosition();
    btVector3 rod_pos5 = rodBody5->getCenterOfMassPosition();

    // btQuaternion rod0Quat = rodBody0->getOrientation();
    // btScalar rod0Angle = rod0Quat.getAngle();
    // btVector3 rod0Axis = rod0Quat.getAxis();
    // btQuaternion rod1Quat = rodBody1->getOrientation();
    // btScalar rod1Angle = rod0Quat.getAngle();
    // btVector3 rod1Axis = rod0Quat.getAxis();
    // btQuaternion rod2Quat = rodBody2->getOrientation();
    // btScalar rod2Angle = rod0Quat.getAngle();
    // btVector3 rod2Axis = rod0Quat.getAxis();
    // btQuaternion rod3Quat = rodBody3->getOrientation();
    // btScalar rod3Angle = rod0Quat.getAngle();
    // btVector3 rod3Axis = rod0Quat.getAxis();
    // btQuaternion rod4Quat = rodBody4->getOrientation();
    // btScalar rod4Angle = rod0Quat.getAngle();
    // btVector3 rod4Axis = rod0Quat.getAxis();
    // btQuaternion rod5Quat = rodBody5->getOrientation();
    // btScalar rod5Angle = rod0Quat.getAngle();
    // btVector3 rod5Axis = rod0Quat.getAxis();

    btTransform rod0Trans = rodBody0->getCenterOfMassTransform();
    btMatrix3x3 rod0Basis = rod0Trans.getBasis();
    rod0Basis.getEulerYPR(yaw0, pitch0, roll0);
    // std::cout << "Yaw0:" << yaw0 << ", Pitch0:" << pitch0 << ", Roll0:" << roll0 << std::endl;
    btTransform rod1Trans = rodBody1->getCenterOfMassTransform();
    btMatrix3x3 rod1Basis = rod1Trans.getBasis();
    rod1Basis.getEulerYPR(yaw1, pitch1, roll1);
    // std::cout << "Yaw1:" << yaw1 << ", Pitch1:" << pitch1 << ", Roll1:" << roll1 << std::endl;
    btTransform rod2Trans = rodBody2->getCenterOfMassTransform();
    btMatrix3x3 rod2Basis = rod2Trans.getBasis();
    rod2Basis.getEulerYPR(yaw2, pitch2, roll2);
    // std::cout << "Yaw0:" << yaw0 << ", Pitch0:" << pitch0 << ", Roll0:" << roll0 << std::endl;
    btTransform rod3Trans = rodBody3->getCenterOfMassTransform();
    btMatrix3x3 rod3Basis = rod3Trans.getBasis();
    rod3Basis.getEulerYPR(yaw3, pitch3, roll3);
    // std::cout << "Yaw1:" << yaw1 << ", Pitch1:" << pitch1 << ", Roll1:" << roll1 << std::endl;
    btTransform rod4Trans = rodBody4->getCenterOfMassTransform();
    btMatrix3x3 rod4Basis = rod4Trans.getBasis();
    rod4Basis.getEulerYPR(yaw4, pitch4, roll4);
    // std::cout << "Yaw0:" << yaw0 << ", Pitch0:" << pitch0 << ", Roll0:" << roll0 << std::endl;
    btTransform rod5Trans = rodBody5->getCenterOfMassTransform();
    btMatrix3x3 rod5Basis = rod5Trans.getBasis();
    rod5Basis.getEulerYPR(yaw5, pitch5, roll5);
    // std::cout << "Yaw1:" << yaw1 << ", Pitch1:" << pitch1 << ", Roll1:" << roll1 << std::endl;

    if (doLog) {
    	// std::cout << rod0Angle << " (" << rod0Axis.x() << ","
    		 // << rod0Axis.y() << "," << rod0Axis.z() << ")" << std::endl;
    	// btVector3 capsule_vel = capsuleBody->getLinearVelocity();
    	
    	// Record state info for simulation playback
    // 	data_out << simTime << ", "  
    // 		<< rod_pos0.x() << ", " << rod_pos0.y() << ", " << rod_pos0.z() << ", " << rod0Angle << ", "
    // 		<< rod0Axis.x() << ", " << rod0Axis.y() << ", " << rod0Axis.z() << ", "
    // 		<< rod_pos1.x() << ", " << rod_pos1.y() << ", " << rod_pos1.z() << ", " << rod1Angle << ", "
    // 		<< rod1Axis.x() << ", " << rod1Axis.y() << ", " << rod1Axis.z() << ", "
    // 		<< rod_pos2.x() << ", " << rod_pos2.y() << ", " << rod_pos2.z() << ", " << rod2Angle << ", "
    // 		<< rod2Axis.x() << ", " << rod2Axis.y() << ", " << rod2Axis.z() << ", "
    // 		<< rod_pos3.x() << ", " << rod_pos3.y() << ", " << rod_pos3.z() << ", " << rod3Angle << ", "
    // 		<< rod3Axis.x() << ", " << rod3Axis.y() << ", " << rod3Axis.z() << ", "
    // 		<< rod_pos4.x() << ", " << rod_pos4.y() << ", " << rod_pos4.z() << ", " << rod4Angle << ", "
    // 		<< rod4Axis.x() << ", " << rod4Axis.y() << ", " << rod4Axis.z() << ", "
    // 		<< rod_pos5.x() << ", " << rod_pos5.y() << ", " << rod_pos5.z() << ", " << rod5Angle << ", "
    // 		<< rod5Axis.x() << ", " << rod5Axis.y() << ", " << rod5Axis.z()
    // 		<< std::endl;

        data_out << simTime << ", "  
         << rod_pos0.x() << ", " << rod_pos0.y() << ", " << rod_pos0.z() << ", "
         << yaw0 << ", " << pitch0 << ", " << roll0 << ", "
         << rod_pos1.x() << ", " << rod_pos1.y() << ", " << rod_pos1.z() << ", "
         << yaw1 << ", " << pitch1 << ", " << roll1 << ", "
         << rod_pos2.x() << ", " << rod_pos2.y() << ", " << rod_pos2.z() << ", "
         << yaw2 << ", " << pitch2 << ", " << roll2 << ", "
         << rod_pos3.x() << ", " << rod_pos3.y() << ", " << rod_pos3.z() << ", "
         << yaw3 << ", " << pitch3 << ", " << roll3 << ", "
         << rod_pos4.x() << ", " << rod_pos4.y() << ", " << rod_pos4.z() << ", "
         << yaw4 << ", " << pitch4 << ", " << roll4 << ", "
         << rod_pos5.x() << ", " << rod_pos5.y() << ", " << rod_pos5.z() << ", "
         << yaw5 << ", " << pitch5 << ", " << roll5
         << std::endl;
    }
}