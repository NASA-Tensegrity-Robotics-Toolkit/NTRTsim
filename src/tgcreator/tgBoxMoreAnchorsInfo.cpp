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
 * @file tgBoxMoreAnchorsInfo.cpp
 * @brief Implementation of class tgBoxMoreAnchorsInfo 
 * @author Drew Sabelhaus, Brian Mirletz, and Ryan Adams
 * @date September 2016
 * $Id$
 */

// This Module
#include "tgBoxMoreAnchorsInfo.h"

// The NTRT Core library
#include "core/tgWorldBulletPhysicsImpl.h"

// The Bullet Physics Library
#include "LinearMath/btVector3.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"

// The C++ Standard Library
#include <algorithm>
#include <stdexcept>
#include <cassert>

// The four constructors. Each just calls the parent's constructor.
tgBoxMoreAnchorsInfo::tgBoxMoreAnchorsInfo(const tgBox::Config& config) : 
  tgBoxInfo(config)
{}

tgBoxMoreAnchorsInfo::tgBoxMoreAnchorsInfo(const tgBox::Config& config,
					   tgTags tags) : 
  tgBoxInfo(config, tags)
{}

tgBoxMoreAnchorsInfo::tgBoxMoreAnchorsInfo(const tgBox::Config& config,
					   const tgPair& pair) :
  tgBoxInfo(config, pair)
{}

tgBoxMoreAnchorsInfo::tgBoxMoreAnchorsInfo(const tgBox::Config& config,
					   tgTags tags, const tgPair& pair) :
  tgBoxInfo(config, tags, pair)
{}

/**
 * Re-define this function to return a tgBoxMoreAnchorsInfo. Compare to parent.
 * Note that getConfig() must be called instead of directly trying to access
 * m_config, since that's a private variable of the parent class.
 */
tgRigidInfo* tgBoxMoreAnchorsInfo::createRigidInfo(const tgPair& pair)
{
  return new tgBoxMoreAnchorsInfo(getConfig(), pair);
}

// Create and return a tgBoxMoreAnchors.
tgModel* tgBoxMoreAnchorsInfo::createModel(tgWorld& world)
{
    // @todo: handle tags -> model
    // @todo: check to make sure the rigidBody has been built
    // @todo: Do we need to use the group here?

    // Just in case it hasn't been done already...
    initRigidBody(world); 
    
    #if (0)
    std::cout << "creating tgBoxMoreAnchors with tags " << getTags() << std::endl; 
    #endif
    
    tgBoxMoreAnchors* box = new tgBoxMoreAnchors(getRigidBody(),
						 getTags(), getLength());

    return box;
}

// Redefine the parent class' method so that a tgBoxMoreAnchorsInfo is returned.
std::set<tgRigidInfo*> tgBoxMoreAnchorsInfo::getLeafRigids() 
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
bool tgBoxMoreAnchorsInfo::isNodeOnBoxSurface(const btVector3& nodeVector) const {
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
  // This is not well-determined within NTRT

  // NOTE that width and height are used in btBoxShape in the same way that
  // radius is used for rods: they are from the center to the outside of the box,
  // NOT from one side of the box to another. 
  
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
    widthContributionToHalfExtents = btVector3( 0, getConfig().width, 0);
    heightContributionToHalfExtents = btVector3( getConfig().height, 0, 0);
  }
  else if( lengthContributionToHalfExtents.y() != 0 ) {
    //DEBUGGING
    std::cout << "Length is in y direction, so width will be in x, "
	      << "and height will be in z. " << std::endl;
    widthContributionToHalfExtents = btVector3( getConfig().width, 0, 0);
    heightContributionToHalfExtents = btVector3( 0, 0, getConfig().height);
  }
  else if( lengthContributionToHalfExtents.x() != 0 ) {
    //DEBUGGING
    std::cout << "Length is in x direction, so width will be in z, "
      	      << "and height will be in y. " << std::endl;
    widthContributionToHalfExtents = btVector3( 0, 0, getConfig().width);
    heightContributionToHalfExtents = btVector3(0, getConfig().height, 0);
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
bool tgBoxMoreAnchorsInfo::containsNode(const btVector3& nodeVector) const {
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
    std::cout << "containsNode: true, is part of pair." << std::endl;
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
      return true;
    }

    std::cout << "containsNode: false." << std::endl;
    return false;
  }
}

/**
 * As of 2016-09-01, use the parent's getContainedNodes method,
 * even though TECHNICALLY there are more nodes in this box.
 * @TODO make this return nodes including those for which containsNode
 * was called and returned true.
 */

/*
std::set<btVector3> tgBoxInfo::getContainedNodes() const {
    std::set<btVector3> contained;
    contained.insert(getFrom());
    contained.insert(getTo());
    return contained;
}
*/
