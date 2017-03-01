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

#ifndef T6_MINI_ROLLING_CONTROLLER_H
#define T6_MINI_ROLLING_CONTROLLER_H

/**
 * @file T6MiniRollingController.h
 * @brief Contains the definition of class T6MiniRollingController.
 * @author Edward Zhu
 * @version 1.0.0
 * $Id$
 */

// This Library
#include "core/tgObserver.h"
#include "core/tgRod.h"
#include "controllers/tgBasicController.h"

// The Model
#include "../models/sixBarMiniModel.h"

// The Bullet Physics Library
#include "LinearMath/btVector3.h"

// The C++ Standard Library
#include <vector>
#include <string>
#include <fstream>
#include <ostream>

// Forward declarations
class sixBarMiniModel;

/**
 * A controller which allows for rolling to a goal triangle or rolling with
 * dead reckoning
 */

class T6MiniRollingController : public tgObserver<sixBarMiniModel>
{
public:
	/**
	  * Configuration structure for setting the mode and goal of the rolling
	  * controller
	  */
	struct Config
	{
	public:
		// Overloaded Config function for two controller modes
		Config (double gravity, const std::string& mode, int face_goal);
		Config (double gravity, const std::string& mode, btVector3 dr_goal);
		Config (double gravity, const std::string& mode, int *path, int pathSize);
		Config (double gravity, const std::string& mode);

		double m_gravity;

		// Use "face" for rolling to a goal triangle, use "dr" for dead reckoning
		std::string m_mode;

		// Goal face to roll to
		int m_face_goal;

		// Goal direction to roll towards, specified as an [x,y,z] vector, height (y)
		// is ignored
		btVector3 m_dr_goal;

		int *m_path;
		int m_path_size;
	};

	/**
	 * Constructor, allows a user to specify their own config
	 */
	T6MiniRollingController(const T6MiniRollingController::Config& config);

	/**
	 * Destructor
	 */
	virtual ~T6MiniRollingController();

	/**
	 * Select controller mode based on configuration. Define the normal vectors for 
	 * all icosahedron faces as well as the adjacency matrix.
	 * @param[in] subject - the model that the controller attaches to
	 */
	virtual void onSetup(sixBarMiniModel& subject);

	/**
	 * Run the controller.
	 * @param[in] subject - the model that the controller attaches to
	 * @param[in] dt - the physics time step from the app file
	 */
	virtual void onStep(sixBarMiniModel& subject, double dt);

	/**
	 * Calculate the gravity vector in the robot frame
	 * @return The direction of gravity in the robot frame
	 */
	btVector3 getRobotGravity();

	/**
	 * Calculate the direction vector in the robot frame
	 * @param[in] dirVectWorld - The direction in world frame to be found in robot frame
	 * @return The direction in the robot frame
	 */
	btVector3 getRobotDir(btVector3 dirVectWorld);

	/**
	 * Check to see if the robot is in contact with the ground
	 * @return A boolean indicating whether or not the robot is on the ground
	 */
	bool checkOnGround();

	/**
	 * Detect which surface of the robot is in contact with the ground
	 * ToDo: test and improve for any arbitrary ground orientation
	 * @return The number of the face which is in contact with the ground
	 */
	int contactSurfaceDetection();

	/**
	 * Detect which surface of the robot is in alignment with the direction of travel
	 * @param[in] travelDir - Direction to travel in
	 * @return The number of the face which is in alignment with the direction of travel
	 */
	int headingSurfaceDetection(btVector3& travelDirection, int currFace);

	/**
	 * Find the shortest path using Dijkstra to get from the start node to the end node
	 * @param[in] adjMat - The adjacency matrix of the robot where each face is representated as a node
	 * @param[in] startNode - The starting node
	 * @param[in] endNode - The destination node
	 * @return A vector containing the sequence of steps to get from the start node to end node
	 */
	std::vector<int> findPath(std::vector< std::vector<int> >& adjMat, 
							  int startNode, int endNode);

	/**
	 * Perform actuation to move robot to desired face
	 * @param[in] dt - Time step
	 * @return A boolean indicating whether or not the step has been completed
	 */
	bool stepToFace(double dt);

	/**
	 * Function for determining whether or not a certain face is a closed triangle on the
	 * robot
	 * @param[in] desFace - Face to check
	 * @return A boolean indicating whether or not the face is a closed triangle
	 */
	bool isClosedFace(int desFace);

	/**
	 * Function for determining whether or not two closed faces are adjacent
	 * @param[in] currFace - Current face
	 * @param[in] desFace - Face to check
	 * @return A boolean indicating whether or not the two closed faces are adjacent
	 */
	bool isAdjacentFace(int currFace, int desFace);

	/**
	 * Function for setting all actuators in a vector to a certain length
	 * @param[in] controllers - Vector of tgBasicController objects
	 * @param[in] actuators - Vector of tgBasicActuator objects
	 * @param[in] setLength - Desired cable length
	 * @param[in] dt - Time step
	 * @return A boolean indicating whether or not the action has completed
	 */
	bool setAllActuators(std::vector<tgBasicController*>& controllers, 
						 std::vector<tgBasicActuator*>& actuators, 
						 double setLength, double dt);

	/**
	 * Function for setting a single actuator to a certain length
	 * @param[in] controller - tgBasicController object
	 * @param[in] actuator - tgBasicActuator object
	 * @param[in] setLength - Desired cable length
	 * @param[in] dt - Time step
	 * @return A boolean indicating whether or not the action has completed
	 */
	bool setSingleActuator(tgBasicController* controller, tgBasicActuator* actuator,
						   double setLength, double dt);

	int resetCounter = 0;
	bool resetFlag = false;

private:
	// Store the configuration data for use later
	Config m_config;
	std::string c_mode;
	int c_face_goal;
	btVector3 c_dr_goal;
	int *c_path;
	int c_path_size;
	int controller_mode;

	// Vector of rigid body objects
	std::vector<btRigidBody*> rodBodies;

	// Payload rigid body object
	btRigidBody* payloadBody;

	// A vector to hold all normal vectors
	std::vector<btVector3> normVects;

	// Gravity vectors
	btVector3 gravVectWorld;

	// Vector to hold path found using Dijkstra's
	std::vector<int> path;

	// Rows of adjacency matrix
	std::vector<int> node0Adj;
	std::vector<int> node1Adj;
	std::vector<int> node2Adj;
	std::vector<int> node3Adj;
	std::vector<int> node4Adj;
	std::vector<int> node5Adj;
	std::vector<int> node6Adj;
	std::vector<int> node7Adj;
	std::vector<int> node8Adj;
	std::vector<int> node9Adj;
	std::vector<int> node10Adj;
	std::vector<int> node11Adj;
	std::vector<int> node12Adj;
	std::vector<int> node13Adj;
	std::vector<int> node14Adj;
	std::vector<int> node15Adj;
	std::vector<int> node16Adj;
	std::vector<int> node17Adj;
	std::vector<int> node18Adj;
	std::vector<int> node19Adj;

	// Vector holding row information of adjacency matrix
	std::vector< std::vector<int> > A;

	std::vector<int> node0AdjClose;
	std::vector<int> node2AdjClose;
	std::vector<int> node5AdjClose;
	std::vector<int> node7AdjClose;
	std::vector<int> node8AdjClose;
	std::vector<int> node10AdjClose;
	std::vector<int> node13AdjClose;
	std::vector<int> node15AdjClose;

	std::vector< std::vector<int> > AClose;

	// Debugging counter
	int counter = 0;

	// Vectors to hold actuators and rods
	std::vector<tgBasicActuator*> actuators;
	std::vector<tgRod*> rods;
	std::vector<tgRod*> payload;

	// Vector to hold controllers for the cables
	std::vector<tgBasicController*> m_controllers;
	
	// Rest length and start length of cables
	double restLength;
	double startLength;

	// Rows of actuation policy matrix
	std::vector<int> node0AP;
	std::vector<int> node1AP;
	std::vector<int> node2AP;
	std::vector<int> node3AP;
	std::vector<int> node4AP;
	std::vector<int> node5AP;
	std::vector<int> node6AP;
	std::vector<int> node7AP;
	std::vector<int> node8AP;
	std::vector<int> node9AP;
	std::vector<int> node10AP;
	std::vector<int> node11AP;
	std::vector<int> node12AP;
	std::vector<int> node13AP;
	std::vector<int> node14AP;
	std::vector<int> node15AP;
	std::vector<int> node16AP;
	std::vector<int> node17AP;
	std::vector<int> node18AP;
	std::vector<int> node19AP;

	// Actuation policy table
	std::vector< std::vector<int> > actuationPolicy;
	std::vector<int> sequence;
	int sequenceIdx = 0;

	// Hold current surface from contact surface detection
	int currSurface = -1;
	int goalSurface = -1;

	int actuatorNum = 1;
	int actuatorNum1 = 1;
	int actuatorNum2 = 1;
	std::vector<bool> actuatorStatus;
	double timer = 0;
	int cableNum;
	int cableNum1;
	int cableNum2;

	int logCounter = 10;

	// Initialize flags
	bool moveComplete = true;
	bool moveComplete1 = true;
	bool moveComplete2 = true;
	bool robotReady = false;
	bool stepFin = true;
	bool goalReached = false;
	bool runPathGen = false;
	bool reorient = false;
	bool drGoalReached = false;
	bool isOnGround;
	int pathIdx = 1;
	int stepIdx = 0;
	bool doLog;

	// Direction to travel
	btVector3 travelDir;
	btVector3 currPos;

	// Initialize filestream and data
	std::ofstream data_out;
	double worldTime = 0;
	int actuatedCable;
	int currentFace;
	double percentChange = 0;

	int mode = 0; // 0: nonsequential 1: simultaneous
};

#endif