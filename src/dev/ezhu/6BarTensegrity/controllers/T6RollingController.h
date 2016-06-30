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
	  * Allow a user to specify their own config
	  */
	T6RollingController(const T6RollingController::Config& config);

	/**
	  * Destructor
	  */
	virtual ~T6RollingController();

	virtual void onSetup(sixBarModel& subject);

	virtual void onStep(sixBarModel& subject, double dt);

	btVector3& getRobotGravity();

	int contactSurfaceDetection();

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

	// Transformation information
	btTransform worldTrans;

	// Gravity vectors
	btVector3 gravVectWorld;
	btVector3 gravVectRobot;

	// Transformation matricies
	btMatrix3x3 robotToWorld;
	btMatrix3x3 worldToRobot;

	// Debugging counter
	int counter;
};

#endif