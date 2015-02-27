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
 * @file tgTouchSensorSphereInfo.cpp
 * @brief Contains the definitions of members of class tgTouchSensorSphereInfo
 * @author Alexander Xydes
 * @date November 2014
 * $Id$
 */

// This module
#include "tgTouchSensorSphereInfo.h"
// This application
#include "tgTouchSensorModel.h"

// The NTRT Core Libary
#include "core/abstractMarker.h"
#include "core/tgBulletUtil.h"
#include "core/tgTagSearch.h"
#include "core/tgBulletUtil.h"
#include "core/tgWorld.h"

// The NTRT Creator Libary
#include "tgcreator/tgNode.h"
#include "tgcreator/tgNodes.h"
#include "tgcreator/tgUtil.h"

// The Bullet Physics library
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "LinearMath/btVector3.h"

tgTouchSensorSphereInfo::tgTouchSensorSphereInfo(const tgSphere::Config& config) :
    tgSphereInfo(config)
{}

tgTouchSensorSphereInfo::tgTouchSensorSphereInfo(const tgSphere::Config& config, tgTags tags) :
    tgSphereInfo(config, tags)
{}

tgTouchSensorSphereInfo::tgTouchSensorSphereInfo(const tgSphere::Config& config, const tgNode &node) :
    tgSphereInfo(config, node)
{}

tgTouchSensorSphereInfo::tgTouchSensorSphereInfo(const tgSphere::Config& config, tgTags tags, const tgNode &node) :
    tgSphereInfo(config, tags, node)
{}

tgRigidInfo* tgTouchSensorSphereInfo::createRigidInfo(const tgNode& node)
{
    return new tgTouchSensorSphereInfo(getConfig(), node);
}

void tgTouchSensorSphereInfo::initRigidBody(tgWorld& world)
{
    if(!getCollisionObject())
    {
		// we want to do this based on group instead the rigid itself; otherwise we throw away autocompounding.
		tgRigidInfo* rigid = getRigidInfoGroup();

		// If we're not using autocompounding, use the rigid body itself.
		// NOTE: This means that auto-compounding can be silently skipped, which means that your parts may not be joined correctly. Do we want that?
        if(rigid == 0)
        {
			rigid = this;
		}

        if (rigid->getCollisionObject() == NULL) // Init only if it doesn't have a btRigidBody (has already been initialized)
		{ 
			btDynamicsWorld& m_dynamicsWorld = tgBulletUtil::worldToDynamicsWorld(world);
			
            btTransform transform = rigid->getTransform();
			btCollisionShape* shape = rigid->getCollisionShape(world);
			
			// Dynamics world will own this
            btPairCachingGhostObject* ghostObject = new btPairCachingGhostObject();
	
            ghostObject->setCollisionShape(shape);
			ghostObject->setWorldTransform(transform);
			ghostObject->setCollisionFlags (btCollisionObject::CF_NO_CONTACT_RESPONSE);
			
			// @todo look up what the second and third arguments of this are
			m_dynamicsWorld.addCollisionObject(ghostObject,btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter|btBroadphaseProxy::DefaultFilter);
			
            rigid->setCollisionObject(ghostObject);
		}
	}
}

tgModel* tgTouchSensorSphereInfo::createModel(tgWorld& world)
{
    // @todo: handle tags -> model
    // @todo: check to make sure the rigidBody has been built
    // @todo: Do we need to use the group here?

    // Just in case it hasn't been done already...
    initRigidBody(world); 
    
//    std::cout << "creating sphere: " << *this << std::endl;
    
    btPairCachingGhostObject* ghostObject = tgCast::cast<btCollisionObject, btPairCachingGhostObject> (getCollisionObject());
    
    tgTouchSensorModel* slimer = new tgTouchSensorModel(ghostObject, world, getTags());

    return slimer;
}
