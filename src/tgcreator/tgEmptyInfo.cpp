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
 * @file tgRodInfo.cpp
 * @brief Implementation of class tgRodInfo 
 * @author Ryan Adams
 * @date January 2014
 * $Id$
 */

// This Module
#include "tgEmptyInfo.h"

// The NTRT Core library
#include "core/tgWorldBulletPhysicsImpl.h"

// The Bullet Physics Library
#include "LinearMath/btVector3.h"

// The C++ Standard Library
#include <algorithm>
#include <stdexcept>
#include <cassert>

tgEmptyInfo::tgEmptyInfo()
{}

tgEmptyInfo::tgEmptyInfo(const tgPair& pair) :
m_pair(pair)
{}

void tgEmptyInfo::initRigidBody(tgWorld& world)
{
}

tgEmptyInfo* tgEmptyInfo::createRigidInfo(const tgPair& pair)
{
    return new tgEmptyInfo(pair);
}

// This function converts a tgRigidInfo ---> tgModel.
tgModel* tgEmptyInfo::createModel(tgWorld& world)
{
    return NULL; // There can be no tgModel for an empty object.
}

btCollisionShape* tgEmptyInfo::getCollisionShape(tgWorld& world) const
{
    if (m_collisionShape == NULL) 
    {
        m_collisionShape = new btCompoundShape(&world);
    }
    return m_collisionShape;
}


double tgEmptyInfo::getMass() const
{
    return 0;
}

btVector3 
tgEmptyInfo::getConnectionPoint(const btVector3& referencePoint,
                   const btVector3& destinationPoint) const
{
    return  getConnectionPoint(referencePoint, destinationPoint, 0);
}

btVector3 
tgEmptyInfo::getConnectionPoint(const btVector3& referencePoint,
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
    // want on the radius of the rod
    
    // When added to any point along the cylinder axis, this will take you
    // to the surface in the direction of the destinationPoint
    
    const btVector3 surfaceVector = directional.rotate(cylinderAxis2, rotation).normalize();

    // Return the the surface point closest to the reference point in the
    // direction of the destination point. 
    return referencePoint + surfaceVector;
}

std::set<tgRigidInfo*> tgEmptyInfo::getLeafRigids() 
{
    std::set<tgRigidInfo*> leaves;
    leaves.insert(this);
    return leaves;
}

std::set<btVector3> tgEmptyInfo::getContainedNodes() const {
    std::set<btVector3> contained;
    contained.insert(getFrom());
    contained.insert(getTo());
    return contained;
}
