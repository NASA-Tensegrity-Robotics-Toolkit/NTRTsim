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

#ifndef CORDE_MODEL
#define CORDE_MODEL

/**
 * @file CordeModel.h
 * @brief Defines structure for the Corde softbody String Model
 * @author Brian Mirletz
 * $Id$
 */

// Bullet Linear Algebra
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"

// The C++ Standard Library
#include <vector>

class CordeModel
{
public:
	CordeModel(btVector3 pos1, btVector3 pos2);
	
	~CordeModel();
	
	void step (btScalar dt);
	
private:
	void computeConstants();

	void stepPrerequisites();

	void computeInternalForces();
	

	struct CordePositionElement
	{
		btVector3 pos;
		btVector3 vel;
		btVector3 force;
		btScalar mass;
	};
	struct CordeQuaternionElement
	{
		btQuaternion q;
		btQuaternion qdot;
		/**
		 * Just a 4x1 vector, but easier to store this way.
		 */
		btQuaternion tprime;
		btVector3 torques;
		btVector3 omega;
	};

	std::vector<CordePositionElement*> m_massPoints;
	std::vector<CordeQuaternionElement*> m_Centerlines;
};
 
 
#endif // CORDE_MODEL
