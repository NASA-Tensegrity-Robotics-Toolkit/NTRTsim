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
//#include "numeric/ublas/matrix.hpp"
//#include "numeric/ublas/io.hpp" 
#include "assign/list_of.hpp"
// Utility Library
#include "../utility.hpp"

T6RollingController::Config::Config (double gravity, const std::string& mode, int face_goal) : 
m_gravity(gravity), m_mode(mode), m_face_goal(face_goal)
{
	assert(m_gravity >= 0);
	assert((m_face_goal >= 0) && (m_face_goal <=7));

	if (m_mode.compare("face") != 0) {
		std::cout << "Config: invalid arguments" << std::endl;
		std::cout << "Usage: first arg is a string for mode ('face' or 'dr'). Second arg is based on mode, if 'face' was used, then an int between 0 and 7 is expected. If 'dr' was used, then a btVector3 is expected" << std::endl;
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
		std::cout << "Usage: first arg is a string for mode ('face' or 'dr'). Second arg is based on mode, if 'face' was used, then an int between 0 and 7 is expected. If 'dr' was used, then a btVector3 is expected" << std::endl;
		std::cout << "Exiting..." << std::endl;
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
	m_controllers.clear();
}

void T6RollingController::onSetup(sixBarModel& subject)
{
	std::cout << "onSetup: " << c_mode << " mode chosen" << std::endl;
	if (c_mode.compare("face") == 0) {
		std::cout << "onSetup: Goal face: " << c_face_goal << std::endl;
		controller_mode = 1;
	}
	else {
		std::cout << "onSetup: Dead reckoning direction: [" << c_dr_goal.x() << ", " 
			<< c_dr_goal.y() << ", " << c_dr_goal.z() << "]" << std::endl;
		controller_mode = 2;
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

	// Retrieve normal vectors from model
	normVects = subject.getNormVects();

	// Find the rest length of the cables
	restLength = subject.face0Edge0.norm();
	//std::cout << "onSetup: Cable rest length: " << restLength << std::endl;

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

	/*
	// Create adjacency matrix (with open triangles connected)
	// 						  Columns: 0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19  // rows:
	node0Adj  = boost::assign::list_of(0)(1)(0)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0)(0); // 0
	node1Adj  = boost::assign::list_of(1)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0)(0)(0)(0); // 1
	node2Adj  = boost::assign::list_of(0)(1)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0); // 2
	node3Adj  = boost::assign::list_of(0)(0)(1)(0)(1)(0)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0); // 3
	node4Adj  = boost::assign::list_of(1)(0)(0)(1)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0); // 4
	node5Adj  = boost::assign::list_of(0)(0)(0)(0)(1)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1); // 5
	node6Adj  = boost::assign::list_of(0)(0)(0)(0)(0)(1)(0)(1)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0); // 6
	node7Adj  = boost::assign::list_of(0)(0)(0)(1)(0)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0); // 7
	node8Adj  = boost::assign::list_of(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0)(1)(0)(0)(0)(0)(0)(0)(1); // 8
	node9Adj  = boost::assign::list_of(0)(0)(0)(0)(0)(0)(1)(0)(1)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0); // 9
	node10Adj = boost::assign::list_of(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(1)(0)(0)(0)(0)(0)(0)(1)(0); // 10
	node11Adj = boost::assign::list_of(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(1)(0)(0)(1)(0)(0)(0)(0); // 11
	node12Adj = boost::assign::list_of(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0)(1)(0)(1)(0)(0)(0)(0)(0)(0); // 12
	node13Adj = boost::assign::list_of(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(1)(0)(1)(0)(0)(0); // 13
	node14Adj = boost::assign::list_of(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(1)(0)(0)(0)(0); // 14
	node15Adj = boost::assign::list_of(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0)(1)(0)(0)(1)(0)(0); // 15
	node16Adj = boost::assign::list_of(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0)(0)(0)(0)(1); // 16
	node17Adj = boost::assign::list_of(0)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0)(1)(0); // 17
	node18Adj = boost::assign::list_of(0)(0)(0)(0)(0)(0)(0)(1)(0)(0)(1)(0)(0)(0)(0)(0)(0)(1)(0)(0); // 18
	node19Adj = boost::assign::list_of(0)(0)(0)(0)(0)(1)(0)(0)(1)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0)(0); // 19
	*/

	// Open triangles not connected
	// 						  Columns: 0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19  // rows:
	node0Adj  = boost::assign::list_of(0)(1)(0)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0)(0); // 0
	node1Adj  = boost::assign::list_of(1)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0); // 1
	node2Adj  = boost::assign::list_of(0)(1)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0); // 2
	node3Adj  = boost::assign::list_of(0)(0)(1)(0)(0)(0)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0); // 3
	node4Adj  = boost::assign::list_of(1)(0)(0)(0)(0)(1)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(0)(1)(0)(0); // 4
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
}

void T6RollingController::onStep(sixBarModel& subject, double dt)
{
	if (dt <= 0.0) {
    	throw std::invalid_argument("onStep: dt is not positive");
  	}

  	if (robotReady == true) {
  		switch (controller_mode) {
  		case 1:
  			{
	  			// Code for face mode
	  			bool isOnGround = checkOnGround();
	  			
	  			// Remove later
	  			int currSurface = contactSurfaceDetection();

	  			/*
	  			if (isOnGround == true && runPathGen == false) {
	  				int currSurface = contactSurfaceDetection();
	  				if (currSurface >= 0) {
	  					path = findPath(A, currSurface, c_face_goal);
	  					utility::printVector(path);
	  				}
	  				runPathGen = true;
	  			}
	  			else if (isOnGround == false && runPathGen == true) {
	  				runPathGen = false;
	  			}
	  			*/

	  			int cable = 8;
	  			m_controllers[cable]->control(dt, 4);
	  			actuators[cable]->moveMotors(dt);
	  			break;
  			}
  		case 2:
  			{
  				// Code for dead reckoning mode
  				break;
  			}
  		}
  	}
  	else {
  		for (size_t i = 0; i < actuators.size(); i++) {
  			m_controllers[i]->control(dt, restLength);
  			actuators[i]->moveMotors(dt);
  		}
  		if (actuators[actuators.size()-1]->getCurrentLength()-restLength < 0.01) {
  			robotReady = true;
  		}
  	}
}

bool T6RollingController::checkOnGround()
{
	bool onGround = false;
	
	btVector3 rodVel = rodBodies[2]->getLinearVelocity();
	double rodSpeed = rodVel.norm();
	if (abs(rodSpeed) < 0.001) onGround = true;

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

  	std::cout << "contactSurfaceDetection: Contact surface: " << currSurface << std::endl;

	return currSurface;
}

btVector3 T6RollingController::getRobotGravity() 
{
	btTransform worldTrans = rodBodies[2]->getWorldTransform();
	btMatrix3x3 robotToWorld = worldTrans.getBasis();
	//std::cout << robotToWorld.getRow(0) << std::endl;
	//std::cout << robotToWorld.getRow(1) << std::endl;
	//std::cout << robotToWorld.getRow(2) << std::endl;
	// The basis of getWorldTransform() returns the rotation matrix from robot frame
	// to world frame. Invert this matrix to go from world to robot frame
	btMatrix3x3 worldToRobot = robotToWorld.inverse();
  	// Transform the gravity vector from world frame to robot frame
  	btVector3 gravVectRobot = worldToRobot * gravVectWorld;
  	//std::cout << "Gravity vector in robot frame: " << gravVectRobot << std::endl;
  	return gravVectRobot;
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
	//std::cout << "Unvisited init: ";
	//utility::printVector(unvisited);
	std::vector<int> visited;
	std::vector<int> distances(nodes, 1000); // Intialize distance vector to some arbitrarily large value
	distances[startNode] = 0;

	// Initialize path vectors
	std::vector<int> prev(nodes, -1); // Initialize path pointer vector to -1
	std::vector<int> pathVect;
	pathVect.push_back(endNode); // Last element in pathVect is the destination node

	while ((endReached == false) && (noSolution == false)) {
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
		if (currNodeFound == false) {
			std::cout << "findPath: No solution found" << std::endl;
			noSolution = true;
		}

		// If node is found, continue with path finding
		if (noSolution == false) {
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