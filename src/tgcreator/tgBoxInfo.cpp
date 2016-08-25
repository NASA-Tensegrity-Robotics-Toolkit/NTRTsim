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

/**
 * This function is used in containsNode to check if a node is on the surface
 * of the box that will be created by this tgBoxInfo.
 * NOTE that is should only work right now for boxes that are orthogonal
 * to the axes of the world: it "should" return false for all rotated boxes.
 * This function is a close copy of Bullet Physics' btBoxShape::isInside.
 */
bool tgBoxInfo::isNodeOnBoxSurface(const btVector3& nodeVector) const {
  // Like btBoxShape::isInside, first calculate the vector from the
  // center of this box to the the node that's on the corner of the
  // box where all coordinates are positive (the "top right" corner.)
  // This vector starts from one node of the box, and goes halfway between
  // the from and the to node (which should be the center of the box...)
  btVector3 boxCenter = getFrom() + ( getTo() - getFrom() )/2;
  // The "half extents" of the box are the x, y, and z dimensions.
  btVector3 lengthContributionToHalfExtents;
  btVector3 widthContributionToHalfExtents;
  btVector3 heightContributionToHalfExtents;
  // HOWEVER, we need to figure out which dimension corresponds to the length,
  // width, and height of the box.
  
  // Length dimension is along the getFrom(), getTo() axis.
  // Subtracting getTo() from getFrom() should result in zero along two axes...
  lengthContributionToHalfExtents = (getTo() - getFrom()) / 2;
  // DEBUGGING:
  std::cout << "Inside isNodeOnBoxSurface: lengthContributionToHalfExtents == "
	    << lengthContributionToHalfExtents << std::endl;
  
  // The coordinates go in a "counterclockwise" sort of direction,
  // so for example if the z direction is length, then width is y and height is x,
  // and if length is y, width is x and height is z.
  if( lengthContributionToHalfExtents.z() != 0 ) {
    //DEBUGGING
    std::cout << "Length is in z direction, so width will be in y, "
	      << "and height will be in x. " << std::endl;
    widthContributionToHalfExtents = btVector3( 0, m_config.width/2, 0);
    heightContributionToHalfExtents = btVector3( m_config.height/2, 0, 0);
  }
  else if( lengthContributionToHalfExtents.y() != 0 ) {
    //DEBUGGING
    std::cout << "Length is in y direction, so width will be in x, "
	      << "and height will be in z. " << std::endl;
    widthContributionToHalfExtents = btVector3( m_config.width/2, 0, 0);
    heightContributionToHalfExtents = btVector3( 0, 0, m_config.height/2);
  }
  else if( lengthContributionToHalfExtents.x() != 0 ) {
    //DEBUGGING
    std::cout << "Length is in x direction, so width will be in z, "
      	      << "and height will be in y. " << std::endl;
    widthContributionToHalfExtents = btVector3( 0, 0, m_config.width/2);
    heightContributionToHalfExtents = btVector3(0, m_config.height/2, 0);
  }
  else {
    throw std::runtime_error("Length of box seems to be zero inside tgBoxInfo, something is very wrong.");
  }
  //DEBUGGING
  std::cout << "Inside isNodeOnBoxSurface: widthContributionToHalfExtents == "
	    << widthContributionToHalfExtents << std::endl;
  std::cout << "Inside isNodeOnBoxSurface: heightContributionToHalfExtents == "
	    << heightContributionToHalfExtents << std::endl;

  // The half-extents are the sum of the contributions in each direction.
  // Here, all the contributions should be in only one direction each!
  btVector3 halfExtents = lengthContributionToHalfExtents
    + widthContributionToHalfExtents
    + heightContributionToHalfExtents;
  
  //DEBUGGING
  if( 1 ){
    std::cout << "Checking tgBoxInfo::isNodeOnBoxSurface for box with "
	      << "pair (" << getFrom() << ", " << getTo() << "), and half-extents "
	      << halfExtents <<", and node vector "
	      << nodeVector << " ..." << std::endl;
  }
  // Point is on box surface if equal to either positive or negative extents
  // in one dimension, and is within the +/- extents in the other two.
  // This is checked by comparing the location of the point with respect to
  // the box center plus or minus the half extents.
  // For now, check inclusion on surface OR inside since that's easier.
  bool result = (nodeVector.x() <= (boxCenter.x() + halfExtents.x())) &&
    (nodeVector.x() >= (boxCenter.x() - halfExtents.x())) &&
    (nodeVector.y() <= (boxCenter.y() + halfExtents.y())) &&
    (nodeVector.y() >= (boxCenter.y() - halfExtents.y())) &&
    (nodeVector.z() <= (boxCenter.z() + halfExtents.z())) &&
    (nodeVector.z() >= (boxCenter.z() - halfExtents.z()));
  //DEBUGGING
  if( 1 ) {
    std::cout << "Result of node surface check: " << result << std::endl;
  }
  return result;
}

/**
 * This function has been enhanced to include checking of either endpoints
 * or inclusion inside the box.
 */
bool tgBoxInfo::containsNode(const btVector3& nodeVector) const {
  // DEBUGGING
  if( 1 ){
    std::cout << "Called containsNode inside tgBoxInfo with pair "
	      << getFrom() << ", " << getTo() << " for nodeVector "
	      << nodeVector << std::endl;
  }
  // The older version of this function:
  //return ((getFrom() - nodeVector).fuzzyZero() || (getTo() - nodeVector).fuzzyZero());

  // Instead, first do that check:
  if( ((getFrom() - nodeVector).fuzzyZero() || (getTo() - nodeVector).fuzzyZero()) ){
    //DEBUGGING:
    std::cout << "containsNode: true." << std::endl;
    return true;
  }
  else {
    // Check if it's inside or on the surface of the box.
    // This requies that m_collisionShape (which is a btBoxShape) to be set!
    // So, check that first.
    // NOTE that m_collisionShape is (almost?) always NULL at this point in the
    // setup of the simulator, so let's create one temporarily
    // for purposes of checking its isInside method.
    //DEBUGGING:
    std::cout << "Node not either of the coordinates in this box's pair. "
	      << "Now checking inside or on surface of box... " << std::endl;
    if( isNodeOnBoxSurface(nodeVector) ) {
      //DEBUGGING
      std::cout << "Node is on box surface, true! " << std::endl;
    }

    std::cout << "containsNode: false." << std::endl;
    return false;
  }
}

std::set<btVector3> tgBoxInfo::getContainedNodes() const {
    std::set<btVector3> contained;
    contained.insert(getFrom());
    contained.insert(getTo());
    return contained;
}
