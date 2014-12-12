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
 * @file tgBasicContactCableInfo.cpp
 * @brief Implementation of class tgBasicActuatorInfo
 * @author Brian Mirletz and Ryan Adams
 * @date October 2014
 * $Id$
 */

#include "tgBasicContactCableInfo.h"

#include "core/tgBulletContactSpringCable.h"

#include "core/tgBulletUtil.h"
#include "core/tgBulletSpringCableAnchor.h"

#include "tgcreator/tgNode.h"


// The Bullet Physics Library
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"

tgBasicContactCableInfo::tgBasicContactCableInfo(const tgBasicActuator::Config& config) : 
m_config(config),
tgConnectorInfo()
{}

tgBasicContactCableInfo::tgBasicContactCableInfo(const tgBasicActuator::Config& config, tgTags tags) : 
m_config(config),
tgConnectorInfo(tags)
{}

tgBasicContactCableInfo::tgBasicContactCableInfo(const tgBasicActuator::Config& config, const tgPair& pair) :
m_config(config),
tgConnectorInfo(pair)
{}
    

tgConnectorInfo* tgBasicContactCableInfo::createConnectorInfo(const tgPair& pair)
{
    return new tgBasicContactCableInfo(m_config, pair);
}

void tgBasicContactCableInfo::initConnector(tgWorld& world)
{
    // Note: tgBulletContactSpringCable holds pointers to things in the world, but it doesn't actually have any in-world representation.
    m_bulletContactSpringCable = createTgBulletContactSpringCable(world);
}

tgModel* tgBasicContactCableInfo::createModel(tgWorld& world)
{
    // Don't have to do anything in the world for a tgBulletContactSpringCable...
    // @todo: set the name based on joined tags, or remove name from the model...
    //std::cout << "tgBasicContactCableInfo::createModel" << std::endl;
    
    // ensure connector has been initialized
    assert(m_bulletContactSpringCable);
    return new tgBasicActuator(m_bulletContactSpringCable, getTags(), m_config);
}

double tgBasicContactCableInfo::getMass() 
{
    // @todo: calculate a mass? tgBulletContactSpringCable doesn't have mass...
    return 0;
}


tgBulletContactSpringCable* tgBasicContactCableInfo::createTgBulletContactSpringCable(tgWorld& world)
{
    //std::cout << "tgBasicActuatorInfo::createtgBulletContactSpringCable()" << std::endl;
    
    //std::cout << "  getFromRigidInfo(): " << getFromRigidInfo() << std::endl;
    //std::cout << "  getFromRigidInfo(): " << getFromRigidInfo()->getRigidInfoGroup() << std::endl;
    
    // @todo: need to check somewhere that the rigid bodies have been set...
    btRigidBody* fromBody = getFromRigidBody();
    btRigidBody* toBody = getToRigidBody();
	
    tgNode from = getFromRigidInfo()->getConnectionPoint(getFrom(), getTo(), m_config.rotation);
    tgNode to = getToRigidInfo()->getConnectionPoint(getTo(), getFrom(), m_config.rotation);
	
	std::vector<tgBulletSpringCableAnchor*> anchorList;
	
	tgBulletSpringCableAnchor* anchor1 = new tgBulletSpringCableAnchor(fromBody, from);
	anchorList.push_back(anchor1);
	
	tgBulletSpringCableAnchor* anchor2 = new tgBulletSpringCableAnchor(toBody, to);
	anchorList.push_back(anchor2);
	
	/// @todo generalize this to n anchors. May take a new info class 
	
	btTransform transform = tgUtil::getTransform(from, to);
	
	btCompoundShape* m_compoundShape = new btCompoundShape(&world);
	
	btTransform t = transform;
	btVector3 origin(0.0, 0.0, 0.0);
	t.setOrigin(origin);
	
	// @todo import this! Only the first two params matter
	btScalar radius = 0.001;
	btScalar length = (from - to).length() / 2.0;
	btBoxShape* box = new btBoxShape(btVector3(radius, length, radius));
	
	m_compoundShape->addChildShape(t, box);
	
	btPairCachingGhostObject* m_ghostObject = new btPairCachingGhostObject();
	
	// Don't double rotate
	transform.setRotation(btQuaternion::getIdentity());
	
    m_ghostObject->setCollisionShape (m_compoundShape);
    m_ghostObject->setWorldTransform(transform);
    m_ghostObject->setCollisionFlags (btCollisionObject::CF_NO_CONTACT_RESPONSE);
	
	// Add ghost object to world
	// @todo tgBulletContactSpringCable handles deleting from world - should it handle adding too?
	btDynamicsWorld& m_dynamicsWorld = tgBulletUtil::worldToDynamicsWorld(world);
	m_dynamicsWorld.addCollisionObject(m_ghostObject,btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter|btBroadphaseProxy::DefaultFilter);
	
    return new tgBulletContactSpringCable(m_ghostObject, world, anchorList, m_config.stiffness, m_config.damping, m_config.pretension);
}
    
