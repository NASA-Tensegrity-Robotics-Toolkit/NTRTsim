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
 * @author Brian Mirletz
 * $Id$
 */

// This module
#include "tgGhostInfo.h"
// This application
#include "tgGhostModel.h"
#include "tgcreator/tgNode.h"
#include "tgcreator/tgNodes.h"
#include "tgcreator/tgPair.h"
#include "tgcreator/tgPairs.h"
// The NTRT Core Libary
#include "core/tgBulletUtil.h"
#include "core/tgTagSearch.h"
#include "tgcreator/tgUtil.h"
#include "core/tgBulletUtil.h"
#include "core/tgWorld.h"


// The Bullet Physics library
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"

tgGhostInfo::tgGhostInfo(const tgBox::Config& config) : 
    tgBoxInfo(config)
{}

tgGhostInfo::tgGhostInfo(const tgBox::Config& config, tgTags tags) : 
    tgBoxInfo(config, tags)
{}

tgGhostInfo::tgGhostInfo(const tgBox::Config& config, const tgPair& pair) :
    tgBoxInfo(config, pair)
{}

tgGhostInfo::tgGhostInfo(const tgBox::Config& config, tgTags tags, const tgPair& pair) :
    tgBoxInfo(config, tags, pair)
{}

tgRigidInfo* tgGhostInfo::createRigidInfo(const tgPair& pair)
{
    return new tgGhostInfo(getConfig(), pair);
}


/// @todo This is the key class to override
void tgGhostInfo::initRigidBody(tgWorld& world)
{
	if(!getCollisionObject()) {

		// we want to do this based on group instead the rigid itself; otherwise we throw away autocompounding.
		tgRigidInfo* rigid = getRigidInfoGroup();

		// If we're not using autocompounding, use the rigid body itself.
		// NOTE: This means that auto-compounding can be silently skipped, which means that your parts may not be joined correctly. Do we want that?
		if(rigid == 0) { 
			rigid = this;
		}

		if (rigid->getCollisionObject() == NULL) // Init only if it doesn't have a btRigidBody (has already been initialized)
		{ 
		
			btDynamicsWorld& m_dynamicsWorld = tgBulletUtil::worldToDynamicsWorld(world);
			
			btTransform transform = rigid->getTransform();
			btCollisionShape* shape = rigid->getCollisionShape(world);
			
			// Dynamics world will own this
			btPairCachingGhostObject* ghostObject = new btPairCachingGhostObject();
	
			ghostObject->setCollisionShape (shape);
			ghostObject->setWorldTransform(transform);
			ghostObject->setCollisionFlags (btCollisionObject::CF_NO_CONTACT_RESPONSE);
			
			// @todo look up what the second and third arguments of this are
			m_dynamicsWorld.addCollisionObject(ghostObject,btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter|btBroadphaseProxy::DefaultFilter);
			
			rigid->setCollisionObject(ghostObject);
		}

	}
}

tgModel* tgGhostInfo::createModel(tgWorld& world)
{
    // @todo: handle tags -> model
    // @todo: check to make sure the rigidBody has been built
    // @todo: Do we need to use the group here?

    // Just in case it hasn't been done already...
    initRigidBody(world); 
    
    #if (0)
    std::cout << "creating box with tags " << getTags() << std::endl; 
    #endif
    
    btPairCachingGhostObject* ghostObject = tgCast::cast<btCollisionObject, btPairCachingGhostObject> (getCollisionObject());
    
    tgGhostModel* slimer = new tgGhostModel(ghostObject, getTags());

    return slimer;
}
