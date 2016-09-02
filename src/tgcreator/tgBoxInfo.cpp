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
 * @file tgBoxInfo.cpp
 * @brief Implementation of class tgBoxInfo 
 * @author Brian Mirletz and Ryan Adams
 * @date September 2014
 * $Id$
 */

// This Module
#include "tgBoxInfo.h"

// The NTRT Core library
#include "core/tgWorldBulletPhysicsImpl.h"

// The Bullet Physics Library
#include "LinearMath/btVector3.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"

// The C++ Standard Library
#include <algorithm>
#include <stdexcept>
#include <cassert>

// @todo: Need to take tags into account...

tgBoxInfo::tgBoxInfo(const tgBox::Config& config) : 
    tgRigidInfo(),
    m_pair(), 
    m_config(config)
{}

tgBoxInfo::tgBoxInfo(const tgBox::Config& config, tgTags tags) : 
    tgRigidInfo(tags),
    m_pair(), 
    m_config(config)
{}

tgBoxInfo::tgBoxInfo(const tgBox::Config& config, const tgPair& pair) :
    tgRigidInfo(pair.getTags()),
    m_pair(pair),
    m_config(config)
{}

tgBoxInfo::tgBoxInfo(const tgBox::Config& config, tgTags tags, const tgPair& pair) :
    tgRigidInfo( tags + pair.getTags() ),
    m_pair(pair),
    m_config(config)    
{}

tgRigidInfo* tgBoxInfo::createRigidInfo(const tgPair& pair)
{
    return new tgBoxInfo(m_config, pair);
}

void tgBoxInfo::initRigidBody(tgWorld& world)
{
    tgRigidInfo::initRigidBody(world);
    assert(m_collisionObject != NULL);
    getRigidBody()->setFriction(m_config.friction);
    getRigidBody()->setRollingFriction(m_config.rollFriction);
    getRigidBody()->setRestitution(m_config.restitution);
}

tgModel* tgBoxInfo::createModel(tgWorld& world)
{
    // @todo: handle tags -> model
    // @todo: check to make sure the rigidBody has been built
    // @todo: Do we need to use the group here?

    // Just in case it hasn't been done already...
    initRigidBody(world); 
    
    #if (0)
    std::cout << "creating box with tags " << getTags() << std::endl; 
    #endif
    
    tgBox* box = new tgBox(getRigidBody(), getTags(), getLength());

    return box;
}

btCollisionShape* tgBoxInfo::getCollisionShape(tgWorld& world) const
{
    if (m_collisionShape == NULL) 
    {
        const double width = m_config.width;
        const double height = m_config.height;
        const double length = getLength();
        // Nominally x, y, z should we adjust here or the transform?
        m_collisionShape =
            new btBoxShape(btVector3(width, length / 2.0, height));
    
        // Add the collision shape to the array so we can delete it later
        tgWorldBulletPhysicsImpl& bulletWorld =
      (tgWorldBulletPhysicsImpl&)world.implementation();
        bulletWorld.addCollisionShape(m_collisionShape);
    }
    return m_collisionShape;
}

double tgBoxInfo::getMass() const
{
    const double length = getLength();
	const double width = m_config.width;
	const double height = m_config.height;
    const double density = m_config.density;
    const double volume =  length * width * height;
    return volume * density;
}

btVector3 
tgBoxInfo::getConnectionPoint(const btVector3& referencePoint,
                   const btVector3& destinationPoint) const
{
    return  getConnectionPoint(referencePoint, destinationPoint, 0);
}

btVector3 
tgBoxInfo::getConnectionPoint(const btVector3& referencePoint,
                   const btVector3& destinationPoint,
                   const double rotation) const
{
    if (referencePoint == destinationPoint)
    {
      throw 
        std::invalid_argument("Destination point is the reference point.");
    }
    // Find the closest point on the radius from the referencePoint
    const btVector3 cylinderAxis = (getTo() - getFrom()).normalize();
    const btVector3 cylinderAxis2 = (getTo() - getFrom()).normalize();
    // Vector from reference point to destination point
    const btVector3 refToDest =
        (referencePoint - destinationPoint).normalize();

    // Find a vector perpendicular to both the cylinder axis and refToDest
    btVector3 rotationAxis = cylinderAxis.cross(refToDest);
    
    // Handle a vector crossed with itself
    if (rotationAxis.length() == 0.0)
    {
        btScalar a = cylinderAxis[0];
        btScalar b = cylinderAxis[1];
        btScalar c = cylinderAxis[2];
        // Find an arbitrary perpendicular vector
        rotationAxis = btVector3(b - c, -a, a).normalize(); 
    }

    const btVector3 directional =
        cylinderAxis.rotate(rotationAxis, -M_PI / 2.0).normalize();

    // Apply one additional rotation so we can end up anywhere we
    // want on the radius of the box
    
    // When added to any point along the cylinder axis, this will take you
    // to the surface in the direction of the destinationPoint
    
    const btVector3 surfaceVector = directional.rotate(cylinderAxis2, rotation).normalize()
                                    * m_config.width;

    // Return the the surface point closest to the reference point in the
    // direction of the destination point. 
    return referencePoint + surfaceVector;
}

std::set<tgRigidInfo*> tgBoxInfo::getLeafRigids() 
{
    std::set<tgRigidInfo*> leaves;
    leaves.insert(this);
    return leaves;
}

std::set<btVector3> tgBoxInfo::getContainedNodes() const {
    std::set<btVector3> contained;
    contained.insert(getFrom());
    contained.insert(getTo());
    return contained;
}
