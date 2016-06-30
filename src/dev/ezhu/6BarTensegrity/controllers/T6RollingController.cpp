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

T6RollingController::Config::Config (double gravity, const std::string& mode, int face_goal) : 
m_gravity(gravity), m_mode(mode), m_face_goal(face_goal)
{
	assert(m_gravity >= 0);
	assert((m_face_goal >= 0) && (m_face_goal <=7));

	if (m_mode.compare("face") != 0) {
		std::cout << "Error: invalid arguments" << std::endl;
		std::cout << "Usage: first arg is a string for mode ('face' or 'dr'). Second arg is based on mode, if 'face' was used, then an int between 0 and 7 is expected. If 'dr' was used, then a btVector3 is expected" << std::endl;
		exit(EXIT_FAILURE);
	}
}

T6RollingController::Config::Config (double gravity, const std::string& mode, btVector3 dr_goal) :
m_gravity(gravity), m_mode(mode), m_dr_goal(dr_goal)
{
	assert(m_gravity >= 0);
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

	gravVectWorld.setX(0.0);
	gravVectWorld.setY(-config.m_gravity);
	gravVectWorld.setZ(0.0);
}

T6RollingController::~T6RollingController()
{

}

void T6RollingController::onSetup(sixBarModel& subject)
{
	std::cout << "T6RollingController::onSetup" << std::endl;

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

	// Retrieve btRigid bodies from model
	sixBarRod0 = subject.rodBodies[0];
	sixBarRod1 = subject.rodBodies[1];
	sixBarRod2 = subject.rodBodies[2];
	sixBarRod3 = subject.rodBodies[3];
	sixBarRod4 = subject.rodBodies[4];
	sixBarRod5 = subject.rodBodies[5];

	// Find all edge vectors of closed triangles
	face0Edge0 = subject.node8 - subject.node4;
	face0Edge1 = subject.node0 - subject.node8;
	face0Edge2 = subject.node4 - subject.node0;

	face2Edge0 = subject.node9 - subject.node0;
	face2Edge1 = subject.node5 - subject.node9;
	face2Edge2 = subject.node0 - subject.node5;

	face5Edge0 = subject.node3 - subject.node4;
	face5Edge1 = subject.node11 - subject.node3;
	face5Edge2 = subject.node4 - subject.node11;

	face7Edge0 = subject.node5 - subject.node3;
	face7Edge1 = subject.node10 - subject.node5;
	face7Edge2 = subject.node3 - subject.node10;

	face8Edge0 = subject.node11 - subject.node7;
	face8Edge1 = subject.node2 - subject.node11;
	face8Edge2 = subject.node7 - subject.node2;

	face10Edge0 = subject.node10 - subject.node2;
	face10Edge1 = subject.node6 - subject.node10;
	face10Edge2 = subject.node2 - subject.node6;

	face13Edge0 = subject.node1 - subject.node7;
	face13Edge1 = subject.node8 - subject.node1;
	face13Edge2 = subject.node7 - subject.node8;

	face15Edge0 = subject.node6 - subject.node1;
	face15Edge1 = subject.node9 - subject.node6;
	face15Edge2 = subject.node1 - subject.node9;

	// Find normal vectors to all faces
	face0Norm = (face0Edge0.cross(face0Edge2)).normalize();
	face1Norm = (face0Edge1.cross(face2Edge0)).normalize();
	face2Norm = (face2Edge0.cross(face2Edge2)).normalize();
	face3Norm = (face7Edge0.cross(face2Edge2)).normalize();
	face4Norm = (face0Edge2.cross(face5Edge0)).normalize();
	face5Norm = (face5Edge0.cross(face5Edge2)).normalize();
	face6Norm = (face7Edge2.cross(face5Edge1)).normalize();
	face7Norm = (face7Edge0.cross(face7Edge2)).normalize();

	face8Norm = (face8Edge0.cross(face8Edge2)).normalize();
	face9Norm = (face8Edge1.cross(face10Edge0)).normalize();
	face10Norm = (face10Edge0.cross(face10Edge2)).normalize();
	face11Norm = (face15Edge0.cross(face10Edge2)).normalize();
	face12Norm = (face8Edge2.cross(face13Edge0)).normalize();
	face13Norm = (face13Edge0.cross(face13Edge2)).normalize();
	face14Norm = (face15Edge2.cross(face13Edge1)).normalize();
	face15Norm = (face15Edge0.cross(face15Edge2)).normalize();

	face16Norm = (face0Edge0.cross(face13Edge2)).normalize();
	face17Norm = (face15Edge1.cross(face2Edge1)).normalize();
	face18Norm = (face7Edge1.cross(face10Edge1)).normalize();
	face19Norm = (face8Edge0.cross(face5Edge2)).normalize();

	// Place all normal vectors into vector
	normVects.push_back(face0Norm);
	normVects.push_back(face1Norm);
	normVects.push_back(face2Norm);
	normVects.push_back(face3Norm);
	normVects.push_back(face4Norm);
	normVects.push_back(face5Norm);
	normVects.push_back(face6Norm);
	normVects.push_back(face7Norm);
	normVects.push_back(face8Norm);
	normVects.push_back(face9Norm);
	normVects.push_back(face10Norm);
	normVects.push_back(face11Norm);
	normVects.push_back(face12Norm);
	normVects.push_back(face13Norm);
	normVects.push_back(face14Norm);
	normVects.push_back(face15Norm);
	normVects.push_back(face16Norm);
	normVects.push_back(face17Norm);
	normVects.push_back(face18Norm);
	normVects.push_back(face19Norm);

	/*
	std::cout << "Face 0: " << face0Norm << std::endl;
	std::cout << "Face 1: " << face1Norm << std::endl;
	std::cout << "Face 2: " << face2Norm << std::endl;
	std::cout << "Face 3: " << face3Norm << std::endl;
	std::cout << "Face 4: " << face4Norm << std::endl;
	std::cout << "Face 5: " << face5Norm << std::endl;
	std::cout << "Face 6: " << face6Norm << std::endl;
	std::cout << "Face 7: " << face7Norm << std::endl;
	std::cout << "Face 8: " << face8Norm << std::endl;
	std::cout << "Face 9: " << face9Norm << std::endl;
	std::cout << "Face 10: " << face10Norm << std::endl;
	std::cout << "Face 11: " << face11Norm << std::endl;
	std::cout << "Face 12: " << face12Norm << std::endl;
	std::cout << "Face 13: " << face13Norm << std::endl;
	std::cout << "Face 14: " << face14Norm << std::endl;
	std::cout << "Face 15: " << face15Norm << std::endl;
	std::cout << "Face 16: " << face16Norm << std::endl;
	std::cout << "Face 17: " << face17Norm << std::endl;
	std::cout << "Face 18: " << face18Norm << std::endl;
	std::cout << "Face 19: " << face19Norm << std::endl;
	*/
}

void T6RollingController::onStep(sixBarModel& subject, double dt)
{
	int currSurface;

	if (dt <= 0.0) {
    	throw std::invalid_argument("dt is not positive");
  	}

  	//btVector3 rod0Pos = sixBarRod0->getCenterOfMassPosition();
  	//std::cout << rod0Pos << std::endl;

  	switch (controller_mode) {
  		case 1:
  			// Code for face mode
  			counter++;
  			if (counter >= 1000) {
  				currSurface = contactSurfaceDetection();
  				counter = 0;
  			}
  			
  			break;
  		case 2:
  			// Code for dead reckoning mode
  			break;
  	}
}

int T6RollingController::contactSurfaceDetection()
{
	double dotProd;
	double maxDotProd = 0;
	int currSurface = -1;
	btVector3& robotGravity = getRobotGravity();
	for (int i = 0; i < normVects.size(); i++) {
		dotProd = robotGravity.dot(normVects[i]);
		//std::cout << dotProd << std::endl;
		if (dotProd > maxDotProd) {
			maxDotProd = dotProd;
			currSurface = i;
		}
  	}

  	if (currSurface == -1) {
  		std::cout << "No surface found" << std::endl;
  	}

  	std::cout << "Contact surface: " << currSurface << std::endl;

	return currSurface;
}

btVector3& T6RollingController::getRobotGravity() {
	worldTrans = sixBarRod2->getWorldTransform();
	robotToWorld = worldTrans.getBasis();
	//std::cout << robotToWorld.getRow(0) << std::endl;
	//std::cout << robotToWorld.getRow(1) << std::endl;
	//std::cout << robotToWorld.getRow(2) << std::endl;
	worldToRobot = robotToWorld.inverse();
  	gravVectRobot = worldToRobot * gravVectWorld;
  	std::cout << "Gravity vector in robot frame: " << gravVectRobot << std::endl;
  	return gravVectRobot;
}