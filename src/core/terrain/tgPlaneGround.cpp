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
 * @file tgPlaneGround.cpp
 * @brief Contains the definition of class tgPlaneGround.
 * @author Atil Iscen
 * $Id$
 */


//This Module
#include "tgPlaneGround.h"

//Bullet Physics
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btTransform.h"

// The C++ Standard Library
#include <cassert>

tgPlaneGround::Config::Config( btVector3 normalVector,
                btScalar friction,
                btScalar restitution,
                btVector3 origin) :
m_normalVector(normalVector),
m_friction(friction),
m_restitution(restitution),
m_origin(origin)
{
    assert((m_friction >= 0.0) && (m_friction <= 1.0));
    assert((m_restitution >= 0.0) && (m_restitution <= 1.0));
}

tgPlaneGround::tgPlaneGround() :
m_config(Config())
{
    // @todo make constructor aux to avoid repeated code
    pGroundShape = new btStaticPlaneShape(m_config.m_normalVector,0);
       
}

tgPlaneGround::tgPlaneGround(const tgPlaneGround::Config& config) :
m_config(config)
{
    pGroundShape = new btStaticPlaneShape(m_config.m_normalVector,0);
}


btRigidBody* tgPlaneGround::getGroundRigidBody() const
{
    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(m_config.m_origin);
    
    btQuaternion orientation;
    orientation.setEuler(0,0,0);
//    orientation.setEuler(m_config.m_eulerAngles[0], // Yaw
//                            m_config.m_eulerAngles[1], // Pitch
//                            m_config.m_eulerAngles[2]); // Roll
    groundTransform.setRotation(orientation);
    
    // Using motionstate is recommended
    // It provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* const pMotionState =
    new btDefaultMotionState(groundTransform);
    
    const btVector3 localInertia(0, 0, 0);

//    btRigidBody::btRigidBodyConstructionInfo const rbInfo(mass, pMotionState, pGroundShape, localInertia);

    btRigidBody* const pGroundBody=new btRigidBody(0.0,pMotionState,pGroundShape,localInertia);
    pGroundBody->setFriction(m_config.m_friction);
    pGroundBody->setRestitution(m_config.m_restitution);

//    btRigidBody* const pGroundBody = new btRigidBody(rbInfo);
    
    return pGroundBody;
}  

