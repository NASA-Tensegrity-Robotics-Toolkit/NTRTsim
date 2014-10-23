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
 * @file tgMultiPointStringInfo.cpp
 * @brief Implementation of class tgLinearStringInfo
 * @author Brian Mirletz and Ryan Adams
 * @date October 2014
 * $Id$
 */

#include "tgMultiPointStringInfo.h"

#include "MuscleNP.h"

#include "core/tgBulletUtil.h"
#include "tgcreator/tgUtil.h"

// The Bullet Physics Library
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"

tgMultiPointStringInfo::tgMultiPointStringInfo(const tgLinearString::Config& config) : 
m_config(config),
tgConnectorInfo()
{}

tgMultiPointStringInfo::tgMultiPointStringInfo(const tgLinearString::Config& config, tgTags tags) : 
m_config(config),
tgConnectorInfo(tags)
{}

tgMultiPointStringInfo::tgMultiPointStringInfo(const tgLinearString::Config& config, const tgPair& pair) :
m_config(config),
tgConnectorInfo(pair)
{}
    

tgConnectorInfo* tgMultiPointStringInfo::createConnectorInfo(const tgPair& pair)
{
    return new tgMultiPointStringInfo(m_config, pair);
}

void tgMultiPointStringInfo::initConnector(tgWorld& world)
{
    // Note: MuscleNP holds pointers to things in the world, but it doesn't actually have any in-world representation.
    m_muscleNP = createMuscleNP(world);
}

tgModel* tgMultiPointStringInfo::createModel(tgWorld& world)
{
    // Don't have to do anything in the world for a MuscleNP...
    // @todo: set the name based on joined tags, or remove name from the model...
    //std::cout << "tgMultiPointStringInfo::createModel" << std::endl;
    
    // ensure connector has been initialized
    assert(m_muscleNP);
    return new tgLinearString(m_muscleNP, getTags(), m_config);
}

double tgMultiPointStringInfo::getMass() 
{
    // @todo: calculate a mass? MuscleNP doesn't have physics...
    return 0;
}


MuscleNP* tgMultiPointStringInfo::createMuscleNP(tgWorld& world)
{
    //std::cout << "tgLinearStringInfo::createMuscleNP()" << std::endl;
    
    //std::cout << "  getFromRigidInfo(): " << getFromRigidInfo() << std::endl;
    //std::cout << "  getFromRigidInfo(): " << getFromRigidInfo()->getRigidInfoGroup() << std::endl;
    
    // @todo: need to check somewhere that the rigid bodies have been set...
    btRigidBody* fromBody = getFromRigidBody();
    btRigidBody* toBody = getToRigidBody();

    btVector3 from = getFromRigidInfo()->getConnectionPoint(getFrom(), getTo(), m_config.rotation);
    btVector3 to = getToRigidInfo()->getConnectionPoint(getTo(), getFrom(), m_config.rotation);
	
	// Dynamics world will own this
	btPairCachingGhostObject* ghostObject = new btPairCachingGhostObject();
	
	btDynamicsWorld& m_dynamicsWorld = tgBulletUtil::worldToDynamicsWorld(world);
	
	assert((to - from).length() != 0);
	
	// Until we get a proper config
	btScalar radius = 0.1;
	
	btTransform transform = tgUtil::getTransform(from, to);
	
	// Consider making this a box. That way when you have N anchors they can all remain inside of the box
	btCollisionShape* collisionShape =
            new btCylinderShape(btVector3(radius, (to - from).length()/2.0, radius));
	ghostObject->setCollisionShape (collisionShape);
	ghostObject->setWorldTransform(transform);
	ghostObject->setCollisionFlags (btCollisionObject::CF_NO_CONTACT_RESPONSE);
	
	// @todo look up what the second and third arguments of this are
	m_dynamicsWorld.addCollisionObject(ghostObject,btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter|btBroadphaseProxy::DefaultFilter);
	
    return new MuscleNP(ghostObject, m_dynamicsWorld.getBroadphase(), fromBody, from, toBody, to, m_config.stiffness, m_config.damping);
}
    
