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
		Config (const std::string& mode, int face_goal);
		Config (const std::string& mode, btVector3 dr_goal);

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

private:
	// Store the configuration data for use later
	Config m_config;
	std::string c_mode;
	int c_face_goal;
	btVector3 c_dr_goal;
	int controller_mode;

	// Rigid body objects
	btRigidBody* sixBarRod0;

};

#endif