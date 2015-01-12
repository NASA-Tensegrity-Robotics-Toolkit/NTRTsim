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
 * @file tgBulletUtil.cpp
 * @brief Contains the definitions of members of class tgBulletUtil
 * $Id$
 */

// This module
#include "tgBulletUtil.h"
// This application
#include "tgWorld.h"
#include "tgWorldBulletPhysicsImpl.h"
// The Bullet Physics library
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btDefaultMotionState.h"

// @todo: Move this to the tgRigidInfo => tgModel step
// NOTE: this is a copy of localCreateRigidBody from the bullet DemoApplication. 
btRigidBody* tgBulletUtil::createRigidBody(btDynamicsWorld* dynamicsWorld, 
                                           float mass, 
                                           const btTransform& startTransform, 
                                           btCollisionShape* shape)
{

    btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0,0,0);
    if (isDynamic)
            shape->calculateLocalInertia(mass,localInertia);

//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

    btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);

    // This is defined in DemoApplication as BT_LARGE_FLOAT 1e30 if using
    // double precision, 1e18.f if using single
    double defaultContactProcessingThreshold = 1.0e30;  // @TODO: What should this be? 

    btRigidBody* body = new btRigidBody(cInfo);
    body->setContactProcessingThreshold(defaultContactProcessingThreshold);

#else
    btRigidBody* body = new btRigidBody(mass,0,shape,localInertia); 
    body->setWorldTransform(startTransform);
#endif//

    dynamicsWorld->addRigidBody(body);

    return body;
}

btDynamicsWorld& tgBulletUtil::worldToDynamicsWorld(const tgWorld& world)
{
  // Fetch the world's implementation.
  tgWorldImpl& impl = world.implementation();
  // Downcast it for Bullet Physics.
  // Avoid dynamic_cast because it is slow and this is called frequently.
  /// @todo Use typeinfo to verify that this is correct.
  tgWorldBulletPhysicsImpl& bulletPhysicsImpl =
    static_cast<tgWorldBulletPhysicsImpl&>(impl);
  btDynamicsWorld& result = bulletPhysicsImpl.dynamicsWorld();
  return result;
}
