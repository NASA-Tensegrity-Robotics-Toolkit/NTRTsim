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
 * @file tgSphereInfo.cpp
 * @brief Implementation of class tgSphereInfo 
 * @author Ryan Adams
 * @date January 2014
 * $Id$
 */

// This Module
#include "tgSphereInfo.h"

// The NTRT Core library
#include "core/tgWorldBulletPhysicsImpl.h"

// The Bullet Physics Library
#include "LinearMath/btVector3.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"

// The C++ Standard Library
#include <algorithm>
#include <stdexcept>
#include <cassert>

// @todo: Need to take tags into account...

tgSphereInfo::tgSphereInfo(const tgSphere::Config& config) : 
    m_node(), 
    m_config(config),
    tgRigidInfo()
{}

tgSphereInfo::tgSphereInfo(const tgSphere::Config& config, tgTags tags) : 
    m_node(), 
    m_config(config),
    tgRigidInfo(tags)
{}

tgSphereInfo::tgSphereInfo(const tgSphere::Config& config, const tgNode& node) :
    m_node(node),
    m_config(config),
    tgRigidInfo(node.getTags())
{}

tgSphereInfo::tgSphereInfo(const tgSphere::Config& config, tgTags tags, const tgNode& node) :
    m_node(node),
    m_config(config),
    tgRigidInfo( tags + node.getTags() )
{}

tgRigidInfo* tgSphereInfo::createRigidInfo(const tgNode& node)
{
    return new tgSphereInfo(m_config, node);
}

void tgSphereInfo::initRigidBody(tgWorld& world)
{
    tgRigidInfo::initRigidBody(world);
    assert(m_collisionObject != NULL);
    getRigidBody()->setFriction(m_config.friction);
    getRigidBody()->setRollingFriction(m_config.rollFriction);
    getRigidBody()->setRestitution(m_config.restitution);
}

tgModel* tgSphereInfo::createModel(tgWorld& world)
{
    // @todo: handle tags -> model
    // @todo: check to make sure the rigidBody has been built
    // @todo: Do we need to use the group here?

    // Just in case it hasn't been done already...
    initRigidBody(world); 
    
    #if (0)
    std::cout << "creating sphere with tags " << getTags() << std::endl; 
    #endif
    
    tgSphere* sphere = new tgSphere(getRigidBody(), getTags());

    return sphere;
}

btCollisionShape* tgSphereInfo::getCollisionShape(tgWorld& world) const
{
    if (m_collisionShape == NULL) 
    {
        const double radius = m_config.radius;
        m_collisionShape =
            new btSphereShape(radius);
    
        // Add the collision shape to the array so we can delete it later
        tgWorldBulletPhysicsImpl& bulletWorld =
      (tgWorldBulletPhysicsImpl&)world.implementation();
        bulletWorld.addCollisionShape(m_collisionShape);
    }
    return m_collisionShape;
}

double tgSphereInfo::getMass() const
{
    const double radius = m_config.radius;
    const double density = m_config.density;
    const double volume =  4.0 / 3.0 * M_PI * radius * radius * radius;
    return volume * density;
}

btVector3 
tgSphereInfo::getConnectionPoint(const btVector3& referencePoint,
                   const btVector3& destinationPoint) const
{
    return  getConnectionPoint(referencePoint, destinationPoint, 0);
}

btVector3 
tgSphereInfo::getConnectionPoint(const btVector3& referencePoint,
                   const btVector3& destinationPoint,
                   const double rotation) const
{
    if (referencePoint == destinationPoint)
    {
      throw 
        std::invalid_argument("Destination point is the reference point.");
    }
    
    if (rotation != 0)
    {
		std::cerr << "Rotation not yet supported due to lack of central axis" << std::endl;
	}
    
    // Get the center point
    const btVector3 startPoint = (getNode());
    // Vector from reference point to destination point
    const btVector3 refToDest =
        (destinationPoint - referencePoint).normalize();

    // Project along the radius to the destination point
    
    const btVector3 surfaceVector = refToDest
                                    * m_config.radius;

    // Return the the surface point closest to the reference point in the
    // direction of the destination point. 
    return startPoint + surfaceVector;
}

std::set<tgRigidInfo*> tgSphereInfo::getLeafRigids() 
{
    std::set<tgRigidInfo*> leaves;
    leaves.insert(this);
    return leaves;
}

std::set<btVector3> tgSphereInfo::getContainedNodes() const {
    std::set<btVector3> contained;
    contained.insert(getNode());
    contained.insert(getNode());
    return contained;
}
