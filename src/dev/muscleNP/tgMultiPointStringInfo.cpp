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
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgNode.h"
#include "tgcreator/tgNodes.h"
#include "tgcreator/tgPair.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"

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
    // @todo: calculate a mass? MuscleNP doesn't have mass...
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
	
    tgNode from = getFromRigidInfo()->getConnectionPoint(getFrom(), getTo(), m_config.rotation);
    tgNode to = getToRigidInfo()->getConnectionPoint(getTo(), getFrom(), m_config.rotation);
	
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
	btDynamicsWorld& m_dynamicsWorld = tgBulletUtil::worldToDynamicsWorld(world);
	m_dynamicsWorld.addCollisionObject(m_ghostObject,btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter|btBroadphaseProxy::DefaultFilter);
	
    return new MuscleNP(m_ghostObject, world, fromBody, from, toBody, to, m_config.stiffness, m_config.damping);
}
    
