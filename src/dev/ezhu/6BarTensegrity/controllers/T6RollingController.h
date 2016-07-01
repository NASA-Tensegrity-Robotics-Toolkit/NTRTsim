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

#ifndef T6_ROLLING_CONTROLLER_H
#define T6_ROLLING_CONTROLLER_H

/**
 * @file T6RollingController.h
 * @brief Contains the definition of class T6RollingController.
 * @author Edward Zhu
 * @version 1.0.0
 * $Id$
 */

// This Library
#include "core/tgObserver.h"

// The Model
#include "../sixBarModel.h"

// The Bullet Physics Library
#include "LinearMath/btVector3.h"

// The C++ Standard Library
#include <vector>
#include <string>

// Forward declarations
class sixBarModel;

/**
 * A controller which allows for rolling to a goal triangle or rolling with
 * dead reckoning
 */

class T6RollingController : public tgObserver<sixBarModel>
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

		double m_gravity;

		// Use "face" for rolling to a goal triangle, use "dr" for dead reckoning
		std::string m_mode;

		// Goal face to roll to, must be between 0 and 7 for the 8 closed triangles
		int m_face_goal;

		// Goal direction to roll towards, specified as an [x,y,z] vector, height (y)
		// is ignored
		btVector3 m_dr_goal;
	};

	/**
	 * Constructor, allows a user to specify their own config
	 */
	T6RollingController(const T6RollingController::Config& config);

	/**
	 * Destructor
	 */
	virtual ~T6RollingController();

	/**
	 * Select controller mode based on configuration. Define the normal vectors for 
	 * all icosahedron faces as well as the adjacency matrix.
	 * @param[in] subject - the model that the controller attaches to
	 */
	virtual void onSetup(sixBarModel& subject);

	/**
	 * Run the controller.
	 * @param[in] subject - the model that the controller attaches to
	 * @param[in] dt - the physics time step from the app file
	 */
	virtual void onStep(sixBarModel& subject, double dt);

	/**
	 * Calculate the gravity vector in the robot frame
	 * @return The direction of gravity in the robot frame
	 */
	btVector3& getRobotGravity();

	/**
	 * Detect which surface of the robot is in contact with the ground
	 * ToDo: test and improve for any arbitrary ground orientation
	 * @return The number of the face which is in contact with the ground
	 */
	int contactSurfaceDetection();

	/**
	 * Find the shortest path using Dijkstra to get from the start node to the end node
	 * @param[in] adjMat - The adjacency matrix of the robot where each face is representated as a node
	 * @param[in] startNode - The starting node
	 * @param[in] endNode - The destination node
	 * @return A vector containing the sequence of steps to get from the start node to end node
	 */
	std::vector<int> findPath(std::vector< std::vector<int> >& adjMat, int startNode, int endNode);

private:
	// Store the configuration data for use later
	Config m_config;
	std::string c_mode;
	int c_face_goal;
	btVector3 c_dr_goal;
	int controller_mode;

	// Rigid body objects
	btRigidBody* sixBarRod0;
	btRigidBody* sixBarRod1;
	btRigidBody* sixBarRod2;
	btRigidBody* sixBarRod3;
	btRigidBody* sixBarRod4;
	btRigidBody* sixBarRod5;

	// Edge vectors of all closed triangles
	btVector3 face0Edge0;
	btVector3 face0Edge1;
	btVector3 face0Edge2;

	btVector3 face2Edge0;
	btVector3 face2Edge1;
	btVector3 face2Edge2;

	btVector3 face5Edge0;
	btVector3 face5Edge1;
	btVector3 face5Edge2;

	btVector3 face7Edge0;
	btVector3 face7Edge1;
	btVector3 face7Edge2;

	btVector3 face8Edge0;
	btVector3 face8Edge1;
	btVector3 face8Edge2;

	btVector3 face10Edge0;
	btVector3 face10Edge1;
	btVector3 face10Edge2;

	btVector3 face13Edge0;
	btVector3 face13Edge1;
	btVector3 face13Edge2;

	btVector3 face15Edge0;
	btVector3 face15Edge1;
	btVector3 face15Edge2;

	// A vector to hold all normal vectors
	std::vector<btVector3> normVects;

	// Normal vectors of all icosahedron faces
	btVector3 face0Norm;
	btVector3 face1Norm;
	btVector3 face2Norm;
	btVector3 face3Norm;
	btVector3 face4Norm;
	btVector3 face5Norm;
	btVector3 face6Norm;
	btVector3 face7Norm;
	btVector3 face8Norm;
	btVector3 face9Norm;
	btVector3 face10Norm;
	btVector3 face11Norm;
	btVector3 face12Norm;
	btVector3 face13Norm;
	btVector3 face14Norm;
	btVector3 face15Norm;
	btVector3 face16Norm;
	btVector3 face17Norm;
	btVector3 face18Norm;
	btVector3 face19Norm;

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

	// Debugging counter
	int counter;
};

#endif