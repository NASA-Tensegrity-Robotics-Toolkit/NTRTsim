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
#include <algorithm>
#include <math.h>
// Boost Matrix Library
#include "assign/list_of.hpp"
// Utility Library
#include "../utility.hpp"

T6RollingController::Config::Config (double gravity, const std::string& mode, int face_goal) : 
m_gravity(gravity), m_mode(mode), m_face_goal(face_goal)
{
	assert(m_gravity >= 0);
	assert((m_face_goal >= 0) && (m_face_goal <= 19));

	if (m_mode.compare("face") != 0) {
		std::cout << "Config: invalid arguments" << std::endl;
		std::cout << "Usage: first arg is a string for mode ('face', 'path', or 'dr'). Second arg is based on mode, if 'face' was used, then an int between 0 and 7 is expected. If 'dr' was used, then a btVector3 is expected" << std::endl;
		std::cout << "Exiting..." << std::endl;
		exit(EXIT_FAILURE);
	}
}

T6RollingController::Config::Config (double gravity, const std::string& mode, btVector3 dr_goal) :
m_gravity(gravity), m_mode(mode), m_dr_goal(dr_goal)
{
	assert(m_gravity >= 0);
	if (mode.compare("dr") != 0) {
		std::cout << "Config: invalid arguments" << std::endl;
		std::cout << "Usage: first arg is a string for mode ('face', 'path', or 'dr'). Second arg is based on mode, if 'face' was used, then an int between 0 and 7 is expected. If 'dr' was used, then a btVector3 is expected" << std::endl;
		std::cout << "Exiting..." << std::endl;
		exit(EXIT_FAILURE);
	}
}

T6RollingController::Config::Config (double gravity, const std::string& mode, int *path, int pathSize) :
m_gravity(gravity), m_mode(mode), m_path(path), m_path_size(pathSize)
{
	assert(m_gravity >= 0);
	if (mode.compare("path") != 0) {
		std::cout << "Config: invalid arguments" << std::endl;
		std::cout << "Usage: first arg is a string for mode ('face', 'path', or 'dr'). Second arg is based on mode, if 'face' was used, then an int between 0 and 7 is expected. If 'dr' was used, then a btVector3 is expected" << std::endl;
		std::cout << "Exiting..." << std::endl;
		exit(EXIT_FAILURE);
	}
}

T6RollingController::T6RollingController(const T6RollingController::Config& config) : m_config(config)
{
	c_mode = config.m_mode;
	c_face_goal = config.m_face_goal;
	c_dr_goal = config.m_dr_goal;
	c_path = config.m_path;
	c_path_size = config.m_path_size;

	gravVectWorld.setX(0.0);
	gravVectWorld.setY(-config.m_gravity);
	gravVectWorld.setZ(0.0);
}

T6RollingController::~T6RollingController()
{
	m_controllers.clear();
}

void T6RollingController::onSetup(sixBarModel& subject)
{
	std::cout << "onSetup: " << c_mode << " mode chosen" << std::endl;
	if (c_mode.compare("face") == 0) {
		std::cout << "onSetup: Goal face: " << c_face_goal << std::endl;
		controller_mode = 1;
		if (!isClosedFace(c_face_goal)) {
			std::cout << "onSetup: Goal face is not a closed face, exiting..." << std::endl;
			exit(EXIT_FAILURE);
		}
	}
	else if (c_mode.compare("dr") == 0) {
		std::cout << "onSetup: Dead reckoning goal: [" << c_dr_goal.x() << ", " 
			<< c_dr_goal.y() << ", " << c_dr_goal.z() << "]" << std::endl;
		controller_mode = 2;
	}
	else if (c_mode.compare("path") == 0) {
		std::cout << "onSetup: Path size is " << c_path_size << " elements" << std::endl;
		std::cout << "onSetup: Path: [";
		for (int i = 0; i < c_path_size-1; i++) {
			std::cout << *(c_path+i) << ", ";
		}
		std::cout << *(c_path+c_path_size-1) << "]" << std::endl;
		controller_mode = 3;
	}
	else {
		std::cout << "onSetup: Controller mode not recognized, exiting..." << std::endl;
		exit(EXIT_FAILURE);
	}

	// Retrieve rods from model
	rods = subject.getAllRods();
	//std::cout << "onSetup: Number of rods: " << rods.size() << std::endl;

	// Convert from tgRod objects to btRigidBody objects
	for (size_t i = 0; i < rods.size(); i++) {
		tgRod* rod = rods[i];
		btRigidBody* rodBody = rod->getPRigidBody();
		rodBodies.push_back(rodBody);
	}

	// Retrive payload body from model
	payload = subject.getPayload();
	tgRod* payloadRod = payload[0];
	payloadBody = payloadRod->getPRigidBody();

	// Retrieve normal vectors from model
	normVects = subject.getNormVects();

	// Open triangles not connected
	// 						  Columns: 0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19  // rows:
	node0Adj  = boost::assign::list_of(0)(1)(0)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0)(0); // 0
	node1Adj  = boost::assign::list_of(1)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0); // 1
	node2Adj  = boost::assign::list_of(0)(1)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0); // 2
	node3Adj  = boost::assign::list_of(0)(0)(1)(0)(0)(0)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0); // 3
	node4Adj  = boost::assign::list_of(1)(0)(0)(0)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0); // 4
	node5Adj  = boost::assign::list_of(0)(0)(0)(0)(1)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1); // 5
	node6Adj  = boost::assign::list_of(0)(0)(0)(0)(0)(1)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0); // 6
	node7Adj  = boost::assign::list_of(0)(0)(0)(1)(0)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0); // 7
	node8Adj  = boost::assign::list_of(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0)(1)(0)(0)(0)(0)(0)(0)(1); // 8
	node9Adj  = boost::assign::list_of(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0); // 9
	node10Adj = boost::assign::list_of(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(1)(0)(0)(0)(0)(0)(0)(1)(0); // 10
	node11Adj = boost::assign::list_of(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0)(0)(0)(1)(0)(0)(0)(0); // 11
	node12Adj = boost::assign::list_of(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0)(0)(0)(1)(0)(0)(0)(0)(0)(0); // 12
	node13Adj = boost::assign::list_of(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(1)(0)(1)(0)(0)(0); // 13
	node14Adj = boost::assign::list_of(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(1)(0)(0)(0)(0); // 14
	node15Adj = boost::assign::list_of(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0)(1)(0)(0)(1)(0)(0); // 15
	node16Adj = boost::assign::list_of(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0)(0)(0)(0)(0); // 16
	node17Adj = boost::assign::list_of(0)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0)(0)(0); // 17
	node18Adj = boost::assign::list_of(0)(0)(0)(0)(0)(0)(0)(1)(0)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0); // 18
	node19Adj = boost::assign::list_of(0)(0)(0)(0)(0)(1)(0)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0); // 19

	A.push_back(node0Adj);
	A.push_back(node1Adj);
	A.push_back(node2Adj);
	A.push_back(node3Adj);
	A.push_back(node4Adj);
	A.push_back(node5Adj);
	A.push_back(node6Adj);
	A.push_back(node7Adj);
	A.push_back(node8Adj);
	A.push_back(node9Adj);
	A.push_back(node10Adj);
	A.push_back(node11Adj);
	A.push_back(node12Adj);
	A.push_back(node13Adj);
	A.push_back(node14Adj);
	A.push_back(node15Adj);
	A.push_back(node16Adj);
	A.push_back(node17Adj);
	A.push_back(node18Adj);
	A.push_back(node19Adj);

	/*
	// Adjacency matrix debug print out
	for (size_t i = 0; i < A.size(); i++) {
		std::vector<int> row = A[i];
		for (size_t j = 0; j < row.size(); j++) {
			std::cout << row[j] << ", ";
		}
		std::cout << std::endl;
	}
	*/

	// Check if adjacency matrix is square
	int rows = A.size();
	int cols = A[1].size();
	if (rows!= cols) {
		std::cout << "onSetup: Adjacency matrix not square, exiting..." << std::endl;
		//exit(EXIT_FAILURE);
	}

	// Create matrix of adjacent closed faces
	node0AdjClose  = boost::assign::list_of(2)(5)(13);
	node2AdjClose  = boost::assign::list_of(0)(7)(15);
	node5AdjClose  = boost::assign::list_of(0)(8)(7);
	node7AdjClose  = boost::assign::list_of(2)(5)(10);
	node8AdjClose  = boost::assign::list_of(5)(10)(13);
	node10AdjClose  = boost::assign::list_of(7)(8)(15);
	node13AdjClose  = boost::assign::list_of(0)(8)(15);
	node15AdjClose  = boost::assign::list_of(2)(10)(13);

	AClose.push_back(node0AdjClose);
	AClose.push_back(node2AdjClose);
	AClose.push_back(node5AdjClose);
	AClose.push_back(node7AdjClose);
	AClose.push_back(node8AdjClose);
	AClose.push_back(node10AdjClose);
	AClose.push_back(node13AdjClose);
	AClose.push_back(node15AdjClose);

	// Set up controllers for the cables
	m_controllers.clear();
	actuators = subject.getAllActuators();
	//std::cout << "onSetup: Number of actuators: " << actuators.size() << std::endl;
	for (size_t i = 0; i < actuators.size(); i++) {
		tgBasicActuator* const pActuator = actuators[i];
		assert(pActuator != NULL);
		tgBasicController* m_lenController = new tgBasicController(pActuator, restLength);
		m_controllers.push_back(m_lenController);
		//std::cout << "onSetup: Cable " << i << ": " << pActuator->getCurrentLength() << std::endl;
	}

	// Find the rest length and start length of the cables
	restLength = actuators[0]->getRestLength();
	startLength = actuators[0]->getStartLength();

	/*
	// Actuation policy table (No policy for open faces)
	// 						 Columns:  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  // rows:
	node0AP  = boost::assign::list_of(-1)( 0)(-1)(-1)(16)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)( 2)(-1)(-1)(-1); // 0
	node1AP  = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1); // 1
	node2AP  = boost::assign::list_of(-1)( 1)(-1)(18)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)( 3)(-1)(-1); // 2
	node3AP  = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1); // 3
	node4AP  = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1); // 4
	node5AP  = boost::assign::list_of(-1)(-1)(-1)(-1)(17)(-1)(12)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(15); // 5
	node6AP  = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1); // 6
	node7AP  = boost::assign::list_of(-1)(-1)(-1)(19)(-1)(-1)(13)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(14)(-1); // 7
	node8AP  = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)( 9)(-1)(-1)(23)(-1)(-1)(-1)(-1)(-1)(-1)(11); // 8
	node9AP  = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1); // 9
	node10AP = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)( 8)(-1)(21)(-1)(-1)(-1)(-1)(-1)(-1)(10)(-1); // 10
	node11AP = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1); // 11
	node12AP = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1); // 12
	node13AP = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(22)(-1)( 5)(-1)( 6)(-1)(-1)(-1); // 13
	node14AP = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1); // 14
	node15AP = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(20)(-1)(-1)( 4)(-1)(-1)( 7)(-1)(-1); // 15
	node16AP = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1); // 16
	node17AP = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1); // 17
	node18AP = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1); // 18
	node19AP = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1); // 19
	*/

	// Actuation policy table (With policy for open faces)
	// 						 Columns:  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  // rows:
	node0AP  = boost::assign::list_of(-1)( 0)(-1)(-1)(16)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)( 2)(-1)(-1)(-1); // 0
	node1AP  = boost::assign::list_of( 2)(-1)( 3)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1); // 1
	node2AP  = boost::assign::list_of(-1)( 1)(-1)(18)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)( 3)(-1)(-1); // 2
	node3AP  = boost::assign::list_of(-1)(-1)( 1)(-1)(-1)(-1)(-1)(13)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1); // 3
	node4AP  = boost::assign::list_of( 0)(-1)(-1)(-1)(-1)(12)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1); // 4
	node5AP  = boost::assign::list_of(-1)(-1)(-1)(-1)(17)(-1)(12)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(15); // 5
	node6AP  = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(15)(-1)( 9)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1); // 6
	node7AP  = boost::assign::list_of(-1)(-1)(-1)(19)(-1)(-1)(13)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(14)(-1); // 7
	node8AP  = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)( 9)(-1)(-1)(23)(-1)(-1)(-1)(-1)(-1)(-1)(11); // 8
	node9AP  = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(11)(-1)(10)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1); // 9
	node10AP = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)( 8)(-1)(21)(-1)(-1)(-1)(-1)(-1)(-1)(10)(-1); // 10
	node11AP = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)( 8)(-1)(-1)(-1)(-1)( 4)(-1)(-1)(-1)(-1); // 11
	node12AP = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)( 9)(-1)(-1)(-1)(-1)( 5)(-1)(-1)(-1)(-1)(-1)(-1); // 12
	node13AP = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(22)(-1)( 5)(-1)( 6)(-1)(-1)(-1); // 13
	node14AP = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)( 6)(-1)( 7)(-1)(-1)(-1)(-1); // 14
	node15AP = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(20)(-1)(-1)( 4)(-1)(-1)( 7)(-1)(-1); // 15
	node16AP = boost::assign::list_of(16)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(22)(-1)(-1)(-1)(-1)(-1)(-1); // 16
	node17AP = boost::assign::list_of(-1)(-1)(18)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(20)(-1)(-1)(-1)(-1); // 17
	node18AP = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(19)(-1)(-1)(21)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1); // 18
	node19AP = boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(17)(-1)(-1)(23)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1); // 19

	actuationPolicy.push_back(node0AP);
	actuationPolicy.push_back(node1AP);
	actuationPolicy.push_back(node2AP);
	actuationPolicy.push_back(node3AP);
	actuationPolicy.push_back(node4AP);
	actuationPolicy.push_back(node5AP);
	actuationPolicy.push_back(node6AP);
	actuationPolicy.push_back(node7AP);
	actuationPolicy.push_back(node8AP);
	actuationPolicy.push_back(node9AP);
	actuationPolicy.push_back(node10AP);
	actuationPolicy.push_back(node11AP);
	actuationPolicy.push_back(node12AP);
	actuationPolicy.push_back(node13AP);
	actuationPolicy.push_back(node14AP);
	actuationPolicy.push_back(node15AP);
	actuationPolicy.push_back(node16AP);
	actuationPolicy.push_back(node17AP);
	actuationPolicy.push_back(node18AP);
	actuationPolicy.push_back(node19AP);

	doLog = false;

	if (doLog) {
		std::string filename = "13InclineRollingDataStiff.txt";
		// Create filestream for data log and open it
		data_out.open(filename.c_str(), std::fstream::out);
		if (!data_out.is_open()) {
			std::cout << "Failed to open output file" << std::endl;
			exit(EXIT_FAILURE);
		}
		else {
			data_out << "SimTime, ActuatedCable, CurrentFace, PercentChange, TankVelX, TankVelY, TankVelZ, TankPosX, TankPosY, TankPosZ" << std::endl << std::endl;
		}
	}
}

void T6RollingController::onStep(sixBarModel& subject, double dt)
{
	if (dt <= 0.0) {
		throw std::invalid_argument("onStep: dt is not positive");
	}
	else {
		worldTime += dt;
	}
	isOnGround = checkOnGround();

	if (robotReady == true) {
		switch (controller_mode) {
		case 1:
			{
				// Code for face mode
				if (resetFlag) {
					resetFlag = !setAllActuators(m_controllers, actuators, restLength, dt);
					resetCounter = 0;
				}
				else {
					if (isOnGround && !runPathGen && stepFin && !resetFlag) {
						currSurface = contactSurfaceDetection();
						runPathGen = true;
						if (currSurface == c_face_goal) {
							goalReached = true;
							std::cout << "onStep: Destination face reached" << std::endl;
						}
						if (currSurface >= 0 && !goalReached) {
							path = findPath(A, currSurface, c_face_goal);
							utility::printVector(path);
						}
						runPathGen = true;
					}
					else if (!isOnGround && runPathGen) {
						runPathGen = false;
					}

					if (currSurface >= 0 && !goalReached) {
						stepFin = stepToFace(dt);
					}
				}
				//std::cout << "isOnGround: " << isOnGround << ", runPathGen: " << runPathGen << ", stepFin: " << stepFin << ", goalreached: " << goalReached << std::endl;
				break;
			}
		case 2:
			{
				// Code for dead reckoning mode
				if (resetFlag) {
					resetFlag = !setAllActuators(m_controllers, actuators, restLength, dt);
					resetCounter = 0;
				}
				else {
					if (isOnGround && stepFin && !resetFlag) {
						currSurface = contactSurfaceDetection();
						currPos = payloadBody->getCenterOfMassPosition();
						currPos.setY(0);
						if (abs(currPos.x()-c_dr_goal.x())<10 && abs(currPos.z()-c_dr_goal.z())<10) {
							std::cout << "onStep: Goal reached" << std::endl;
							drGoalReached = true;
						}
						if (!runPathGen && !drGoalReached) {
							travelDir = (c_dr_goal - currPos).normalize();
							std::cout << "onStep: Current position: " << currPos << std::endl;
							std::cout << "onStep: Desired position: " << c_dr_goal << std::endl;
							std::cout << "onStep: Travel direction: " << travelDir << std::endl;
							goalSurface = headingSurfaceDetection(travelDir, currSurface);
							path = findPath(A, currSurface, goalSurface);
							utility::printVector(path);
							runPathGen = true;
						}
						if (!drGoalReached) {
							runPathGen = false;
						}
					}
					if (currSurface >= 0 && !drGoalReached) {
						stepFin = stepToFace(dt);
					}
				}
				break;
			}
		case 3:
			{
				// Code for path mode
				if (resetFlag) {
					resetFlag = !setAllActuators(m_controllers, actuators, restLength, dt);
					currentFace = contactSurfaceDetection();
					resetCounter = 0;
				}
				else {
					if (isOnGround && !runPathGen && stepFin && !resetFlag) {
						if (stepIdx == 13) {
							//exit(EXIT_SUCCESS);
						}
						currSurface = contactSurfaceDetection();
						runPathGen = true;
						//std::cout << pathIdx << std::endl;
						if (pathIdx == c_path_size) {
							pathIdx = 0;
							//goalReached = true;
							//std::cout << "onStep: Destination reached" << std::endl;
						}
						if (currSurface >= 0 && !goalReached) {
							//std::cout << currSurface << ", " << pathIdx << ", " << *(c_path+pathIdx) << std::endl;
							path = findPath(A, currSurface, *(c_path+pathIdx));
							pathIdx++;
							stepIdx++;
							utility::printVector(path);
						}
						runPathGen = true;
					}
					else if (!isOnGround && runPathGen) {
						runPathGen = false;
					}

					if (currSurface >= 0 && !goalReached) {
						// stepFin = stepToFace(currSurface, path[1], dt);
						stepFin = stepToFace(dt);
					}
				}
				break;
			}
		}
	}
	else robotReady = setAllActuators(m_controllers, actuators, restLength, dt);
	
	if (doLog) {
		btVector3 payload_vel = payloadBody->getLinearVelocity();
	    btVector3 payload_pos = payloadBody->getCenterOfMassPosition();
	    percentChange = (actuators[actuatedCable]->getCurrentLength()-startLength)/startLength;
	    data_out << worldTime << ", " << actuatedCable << ", " << currentFace << ", " << percentChange << ", " << payload_vel.x() << ", " << payload_vel.y() << ", " << payload_vel.z() << ", " << payload_pos.x() << ", " << payload_pos.y() << ", " << payload_pos.z() << std::endl;
	}
}

bool T6RollingController::checkOnGround()
{
	bool onGround = false;
	
	// btVector3 rodVel = rodBodies[2]->getLinearVelocity();
	// double rodSpeed = rodVel.norm();
	// if (abs(rodSpeed) < 0.001) onGround = true;

	btVector3 payloadVel = payloadBody->getLinearVelocity();
	double payloadSpeed = payloadVel.norm();
	if (abs(payloadSpeed) < 0.001) onGround = true;

	return onGround;
}

int T6RollingController::contactSurfaceDetection()
{
	// Initialize variables
	double dotProd;
	double maxDotProd = 0;
	int currSurface = -1;

	// Get the gravity vector
	btVector3 robotGravity = getRobotGravity();

	// Find the dot product between the gravity vector and each face
	// As all normal vectors point away from the center of the robot,
	// The larger dot product indicates better alignment
	for (size_t i = 0; i < normVects.size(); i++) {
		dotProd = robotGravity.dot(normVects[i]);
		//std::cout << dotProd << std::endl;
		if (dotProd > maxDotProd) {
			maxDotProd = dotProd;
			currSurface = i;
		}
	}

	// Catch all error state
	if (currSurface == -1) {
		std::cout << "contactSurfaceDetection: No surface found" << std::endl;
	}

	// std::cout << "contactSurfaceDetection: Contact surface: " << currSurface << std::endl;

	return currSurface;
}

int T6RollingController::headingSurfaceDetection(btVector3& travelDirWorld, int currFace)
{
	// Initialize variables
	double dotProd;
	double maxDotProd = 0;
	int goalSurface = -1;

	// Get the direction vector in robot frame
	btVector3 travelDirRobot = getRobotDir(travelDirWorld);

	// Find the dot product between the heading vector and each face
	// As all normal vectors point away from the center of the robot,
	// The larger dot product indicates better alignment
	for (size_t i = 0; i < normVects.size(); i++) {
		// if (isAdjacentFace(currFace, i)) {
		if (isClosedFace(i)) {
			dotProd = travelDirRobot.dot(normVects[i]);
			//std::cout << dotProd << std::endl;
			if (dotProd > maxDotProd) {
				maxDotProd = dotProd;
				goalSurface = i;
			}
		}
	}

	// Catch all error state
	if (goalSurface == -1) {
		std::cout << "headingSurfaceDetection: No surface found" << std::endl;
	}

	std::cout << "headingSurfaceDetection: Goal surface: " << goalSurface << std::endl;

	return goalSurface;
}

btVector3 T6RollingController::getRobotGravity() 
{
	btTransform worldTrans = rodBodies[2]->getWorldTransform();
	btMatrix3x3 robotToWorld = worldTrans.getBasis();
	// The basis of getWorldTransform() returns the rotation matrix from robot frame
	// to world frame. Invert this matrix to go from world to robot frame
	btMatrix3x3 worldToRobot = robotToWorld.inverse();
	// Transform the gravity vector from world frame to robot frame
	btVector3 gravVectRobot = worldToRobot * gravVectWorld;
	//std::cout << "Gravity vector in robot frame: " << gravVectRobot << std::endl;
	return gravVectRobot;
}

btVector3 T6RollingController::getRobotDir(btVector3 dirVectWorld) 
{
	btTransform worldTrans = rodBodies[2]->getWorldTransform();
	btMatrix3x3 robotToWorld = worldTrans.getBasis();
	// The basis of getWorldTransform() returns the rotation matrix from robot frame
	// to world frame. Invert this matrix to go from world to robot frame
	btMatrix3x3 worldToRobot = robotToWorld.inverse();
	// Transform the gravity vector from world frame to robot frame
	btVector3 dirVectRobot = (worldToRobot * dirVectWorld).normalize();
	//std::cout << "Gravity vector in robot frame: " << gravVectRobot << std::endl;
	return dirVectRobot;
}

std::vector<int> T6RollingController::findPath(std::vector< std::vector<int> >& adjMat, int startNode, int endNode) 
{
	// Check validity of start and end nodes
	int nodes = adjMat.size();
	if (startNode > nodes) {
		std::cout << "findPath: Start node out of bounds, exiting..." << std::endl;
		exit(EXIT_FAILURE);
	}
	else if (endNode > nodes) {
		std::cout << "findPath: End node out of bounds, exiting..." << std::endl;
		exit(EXIT_FAILURE);
	}

	// Initialize status flags for reached destination or no solution
	bool endReached = false;
	bool noSolution = false;

	if (endNode == startNode) {
		endReached = true;
	}

	// Create vectors to hold unvisited and visited sets and distances
	// the unvisited set initializs with all nodes and the visited set
	// initializes as an empty vector
	std::vector<int> unvisited;
	for (int i = 0; i < nodes; i++) {
		unvisited.push_back(i);
	}
	//utility::printVector(unvisited);
	std::vector<int> visited;
	std::vector<int> distances(nodes, 1000); // Intialize distance vector to some arbitrarily large value
	distances[startNode] = 0;

	// Initialize path vectors
	std::vector<int> prev(nodes, -1); // Initialize path pointer vector to -1
	std::vector<int> pathVect;
	pathVect.push_back(endNode); // Last element in pathVect is the destination node

	while (!endReached && !noSolution) {
		// Initialize variables
		bool currNodeFound = false;
		int currNode = -1;
		int minDist = 1000;

		// Find next unvisited node with the shortest tentative distance
		for (size_t uvNode_idx = 0; uvNode_idx < unvisited.size(); uvNode_idx++) {
			if (distances[unvisited[uvNode_idx]] < minDist) {
				minDist = distances[unvisited[uvNode_idx]];
				currNode = unvisited[uvNode_idx];
				currNodeFound = true;
			}
		}
		//std::cout << "Current node : " << currNode << std::endl;

		// Check if new node is found
		if (!currNodeFound) {
			std::cout << "findPath: No solution found" << std::endl;
			noSolution = true;
		}

		// If node is found, continue with path finding
		if (!noSolution) {
			// Extract row corresponding to the current node
			std::vector<int> currRow = adjMat[currNode];
			std::vector<int> neighbors;
			std::vector<int> weights;
			// Find its neighbors and their corresponding weights
			for (size_t col_idx = 0; col_idx < currRow.size(); col_idx++) {
				if (currRow[col_idx] > 0) {
					neighbors.push_back(col_idx);
					weights.push_back(currRow[col_idx]);
				}
			}
			//utility::printVector(neighbors);
			//utility::printVector(weights);
			// Check if neighbors have been visited already, if not calculate distance
			for (size_t neigh_idx = 0; neigh_idx < neighbors.size(); neigh_idx++) {
				if (find(visited.begin(), visited.end(), neighbors[neigh_idx]) != visited.end()) {
					continue;
				}
				else if (neighbors[neigh_idx] == endNode) {
					prev[endNode] = currNode;
					endReached = true;
					break;
				}
				else {
					int tentativeDistance = distances[currNode] + weights[neigh_idx];
					if (tentativeDistance < distances[neighbors[neigh_idx]]) {
						distances[neighbors[neigh_idx]] = tentativeDistance;
						prev[neighbors[neigh_idx]] = currNode;
					}
				}
			}

			// Finished investigating current node, move it from unvisted to visited
			visited.push_back(currNode);
			unvisited.erase(find(unvisited.begin(), unvisited.end(), currNode));
			//utility::printVector(visited);
			//utility::printVector(unvisited);
		}
	}

	int node = endNode;
	while (node != startNode) {
		pathVect.insert(pathVect.begin(), prev[node]);
		node = prev[node];
	}

	//std::cout << "End Reached: " << endReached << ", No Solution: " << noSolution << std::endl;
	return pathVect;
}

bool T6RollingController::stepToFace(double dt)
{
	// Initialize flags
	bool stepFinished = false;
	bool isOnPath = false;
	// Length for cables to retract to
	//double controlLength = 0.2;
	double controlLength = restLength * 0;
	
	int cableToActuate = -1;
	// Get which cable to actuate from actuation policy table
	if (path.size() > 1) {
		cableToActuate = actuationPolicy[path[0]][path[1]];
		actuatedCable = cableToActuate;

		// Find current face
		int currFace = contactSurfaceDetection();
		currentFace = currFace;

		// Perform actuation from one closed face to another
		if (isClosedFace(path[0])) {
			if (cableToActuate >= 0) {
				// path[0] is current face, path[1] is the adjacent open face, 
				// path[2] is the next closed face
				// Check if the robot has reached the next closed face
				if (currFace != path[2]) {
					m_controllers[cableToActuate]->control(dt, controlLength);
					actuators[cableToActuate]->moveMotors(dt);
					// std::cout << "stepToFace: (Closed -> Closed) Stepping..." << std::endl;
					resetCounter++;
					if (resetCounter > 3.0/dt) resetFlag = true;
				}
				// If it has, return all cables to rest length
				else {
					resetFlag = true;
					path.erase(path.begin(),path.begin()+2);
					utility::printVector(path);
					if (path.size() == 1) {
						stepFinished = true;
					}
				}
			}
			// Triggers if element called from actuation policy table is -1
			else {
				std::cout << "stepToFace: No actuation scheme available, exiting..." << std::endl;
				//exit(EXIT_FAILURE);
			}
		}
		// Perfom actuation to get from an open face to a closed face
		else {
			if (cableToActuate >= 0) {
				// Check to see if robot has reached a closed face
				if (!isClosedFace(currFace)) {
					m_controllers[cableToActuate]->control(dt, controlLength);
					actuators[cableToActuate]->moveMotors(dt);
					// std::cout << "stepToFace: (Open -> Closed) Stepping..." << std::endl;
					resetCounter++;
					if (resetCounter > 3.0/dt) resetFlag = true;
				}
				// If it has, return all cables to rest length
				else {
					resetFlag = true;
					path.erase(path.begin());
					utility::printVector(path);
					stepFinished = true;
				}
			}
			// Triggers if element called from actuation policy table is -1
			else {
				std::cout << "stepToFace: No actuation scheme available, exiting..." << std::endl;
				//exit(EXIT_FAILURE);
			}
		}
	}
	else stepFinished = true;

	
	
	return stepFinished;
}

bool T6RollingController::isClosedFace(int desFace)
{
	bool isClosedFace = false;
	std::vector<int> closedFaces;
	closedFaces  = boost::assign::list_of(0)(2)(5)(7)(8)(10)(13)(15);

	for (size_t i = 0; i < closedFaces.size(); i++) {
		if (desFace == closedFaces[i]) {
			isClosedFace = true;
		}
	}

	return isClosedFace;
}

bool T6RollingController::isAdjacentFace(int currFace, int desFace)
{
	bool isAdjacentFace = false;

	if (find(AClose[currFace].begin(), AClose[currFace].end(), desFace) != AClose[currFace].end()) {
		isAdjacentFace = true;
	}

	return isAdjacentFace;
}

bool T6RollingController::setAllActuators(std::vector<tgBasicController*>& controllers, 
										  std::vector<tgBasicActuator*>& actuators, 
										  double setLength, double dt)
{
	bool returnFin = true;
	for (size_t i = 0; i < actuators.size(); i++) {
		controllers[i]->control(dt, setLength);
		actuators[i]->moveMotors(dt);
		if (actuators[i]->getRestLength()-setLength > 0.01) {
			returnFin = false;
		}
		if (actuators[i]->getRestLength()-setLength < -0.01) {
			returnFin = false;
		}
	}
	// std::cout << "Resetting Cable Lengths " << std::endl;
	return returnFin;
}
