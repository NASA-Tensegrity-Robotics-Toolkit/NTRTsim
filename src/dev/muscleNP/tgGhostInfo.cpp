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
 * @file tgGhostInfo.cpp
 * @brief Contains the definitions of members of class tgGhostInfo
 * @author Ryan Adams
 * $Id$
 */

// This module
#include "tgGhostInfo.h"
// This application
#include "tgNode.h"
#include "tgNodes.h"
#include "tgPair.h"
#include "tgPairs.h"
// The NTRT Core Libary
#include "core/tgBulletUtil.h"
#include "core/tgTagSearch.h"
#include "tgUtil.h"
#include "core/tgBulletUtil.h"
#include "core/tgWorld.h"


// The Bullet Physics library
#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/btDynamicsWorld.h"


/// @todo This is the key class to override
void tgGhostInfo::initRigidBody(tgWorld& world)
{
	if(!getRigidBody()) {

		// we want to do this based on group instead the rigid itself; otherwise we throw away autocompounding.
		tgRigidInfo* rigid = getRigidInfoGroup();

		// If we're not using autocompounding, use the rigid body itself.
		// NOTE: This means that auto-compounding can be silently skipped, which means that your parts may not be joined correctly. Do we want that?
		if(rigid == 0) { 
			rigid = this;
		}

		if (rigid->getRigidBody() == NULL) { // Init only if it doesn't have a btRigidBody (has already been initialized)

			double mass = rigid->getMass();
			btTransform transform = rigid->getTransform();
			btCollisionShape* shape = rigid->getCollisionShape(world);
			
			btRigidBody* body = 
	  tgBulletUtil::createRigidBody(&tgBulletUtil::worldToDynamicsWorld(world),
					mass,
					transform,
					shape);

			rigid->setRigidBody(body);
		}
	}
}

