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

#ifndef CORDE_COLLISION_OBJECT
#define CORDE_COLLISION_OBJECT

/**
 * @file CordeCollisionObject.h
 * @brief Interface Between Corde Model and Bullet
 * @author Brian Mirletz
 * $Id$
 */

// Corde Physics
#include "CordeModel.h"

// Bullet Physics
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

// Bullet Linear Algebra
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"

// The C++ Standard Library
#include <vector>

class CordeCollisionObject : public CordeModel, public btCollisionObject
{
public:

	CordeCollisionObject(std::vector<btVector3>& centerLine, CordeModel::Config& Config);
	
	virtual ~CordeCollisionObject();

};
 
 
#endif // CORDE_MODEL
