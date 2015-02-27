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
 * @file tgTouchSensorModel.cpp
 * @brief Contains the definitions of members of class tgRod
 * @author Brian Mirletz and Ryan Adams
 * $Id$
 */

// This module
#include "tgTouchSensorModel.h"

// The NTRT Core library
#include "core/tgBulletUtil.h"
#include "core/tgModelVisitor.h"
#include "core/tgWorld.h"
#include "core/tgWorldBulletPhysicsImpl.h"
#include "core/abstractMarker.h"

// The NTRT Creator library
#include "tgcreator/tgUtil.h"

// The Bullet Physics library
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"

// The C++ Standard Library
#include <cassert>
#include <stdexcept>

tgTouchSensorModel::tgTouchSensorModel(btPairCachingGhostObject* pGhostObject,
    tgWorld& world,
    const tgTags& tags) :
tgModel(tags),
m_world(world),
m_overlappingPairCache(tgBulletUtil::worldToDynamicsWorld(world).getBroadphase()),
m_dispatcher(tgBulletUtil::worldToDynamicsWorld(world).getDispatcher()),
m_pGhostObject(pGhostObject),
m_bContact(false)
{
    if (pGhostObject == NULL)
    {
            throw std::invalid_argument("Pointer to ghostObject is NULL");
    }

    // Postcondition
    assert(invariant());
    assert(m_pGhostObject == pGhostObject);
}

tgTouchSensorModel::~tgTouchSensorModel()
{
    btDynamicsWorld& dynamicsWorld = tgBulletUtil::worldToDynamicsWorld(m_world);
    dynamicsWorld.removeCollisionObject(m_pGhostObject);
    delete m_pGhostObject;
}

void tgTouchSensorModel::teardown()
{
    tgModel::teardown();
}

void tgTouchSensorModel::onVisit(const tgModelVisitor& v) const
{
//    v.render(*this);
}

void tgTouchSensorModel::step(double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive.");
    }
    else
    {
        updatePosition();
        checkCollisions();
        tgModel::step(dt);  // Step any children

        if (m_bContact)
        {
//            std::cerr << toString() << std::endl;
        }
        else
        {
        }
    }
}

void tgTouchSensorModel::updatePosition()
{
    m_bContact = false;
    std::vector<abstractMarker> markers = getMarkers();
    if (markers.size() == 1)
        m_pGhostObject->setWorldTransform(btTransform(btQuaternion::getIdentity(),markers[0].getWorldPosition()));
}

void tgTouchSensorModel::checkCollisions()
{
    btManifoldArray	m_manifoldArray;
    btVector3 m_touchingNormal;

    // Only caches the pairs, they don't have a lot of useful information
    btBroadphasePairArray& pairArray = m_pGhostObject->getOverlappingPairCache()->getOverlappingPairArray();
    int numPairs = pairArray.size();

    int numContacts = 2;

    for (int i=0;i<numPairs;i++)
    {
        m_manifoldArray.clear();

        const btBroadphasePair& pair = pairArray[i];

        // The real broadphase's pair cache has the useful info
        btBroadphasePair* collisionPair = m_overlappingPairCache->getOverlappingPairCache()->findPair(pair.m_pProxy0,pair.m_pProxy1);

        btCollisionObject* obj0 = static_cast<btCollisionObject*>(collisionPair->m_pProxy0->m_clientObject);
        btCollisionObject* obj1 = static_cast<btCollisionObject*>(collisionPair->m_pProxy1->m_clientObject);

        if (collisionPair->m_algorithm)
            collisionPair->m_algorithm->getAllContactManifolds(m_manifoldArray);

        for (int j=0;j<m_manifoldArray.size();j++)
        {
            btPersistentManifold* manifold = m_manifoldArray[j];
            btScalar directionSign = manifold->getBody0() == m_pGhostObject ? btScalar(-1.0) : btScalar(1.0);

            for (int p=0;p<manifold->getNumContacts();p++)
            {
                const btManifoldPoint& pt = manifold->getContactPoint(p);

                btScalar dist = pt.getDistance();

                if (dist <= 0.0)
                {

                    m_touchingNormal = pt.m_normalWorldOnB * directionSign;

                    btVector3 pos = directionSign < 0 ? pt.m_positionWorldOnB : pt.m_positionWorldOnA;

                    btRigidBody* rb = NULL;

                    if (manifold->getBody0() == m_pGhostObject)
                    {
                        if (manifold->getBody1() == obj1)
                            rb = btRigidBody::upcast(obj1);
                        else if (manifold->getBody1() == obj0)
                            rb = btRigidBody::upcast(obj0);
                        else
                        {
                            throw std::runtime_error("Can't find the right object!!");
                        }
                    }
                    else
                    {
                        if (manifold->getBody0() == obj0)
                            rb = btRigidBody::upcast(obj0);
                        else if (manifold->getBody0() == obj1)
                            rb = btRigidBody::upcast(obj1);
                        else
                        {
                            throw std::runtime_error("Can't find the right object!!");
                        }
                    }

                    if(rb)
                    {
                        if (isRBIgnored(rb))
                        {
                            continue;
                        }
                        else{
//                            std::cerr << "Contact normal: " << m_touchingNormal << std::endl;
//                            std::cerr << "TouchSensor Contact!!!!!! " << rb->getCollisionShape()->getShapeType() << "\n";
                            m_bContact = true;
                        }
                    }
                }
            }
        }

    }
}

bool tgTouchSensorModel::isRBIgnored(const btCollisionObject *_rb)
{
    if (_rb == m_pGhostObject) return true;
    for (size_t i=0; i<m_IgnoredObjects.size(); i++)
    {
        if (_rb == m_IgnoredObjects[i])
            return true;
    }
}

btPairCachingGhostObject* tgTouchSensorModel::getPGhostObject()
{
    return m_pGhostObject;
}

std::vector<const btCollisionObject*> tgTouchSensorModel::getIgnoredObjects()
{
    return m_IgnoredObjects;
}

void tgTouchSensorModel::addIgnoredObject(const btCollisionObject *_objToIgnore)
{
    m_IgnoredObjects.push_back(_objToIgnore);
}

void tgTouchSensorModel::addMarker(abstractMarker &_a)
{
    tgModel::addMarker(_a);
    addIgnoredObject(_a.getAttachedBody());
}

bool tgTouchSensorModel::isTouching()
{
    return m_bContact;
}

btVector3 tgTouchSensorModel::centerOfMass() const
{
  btTransform transform = m_pGhostObject->getWorldTransform();
  const btVector3& result = transform.getOrigin();

  // Return a copy
  return result;
}

bool tgTouchSensorModel::invariant() const
{
  return
    (m_pGhostObject != NULL);
}
