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
 * @author Drew Sabelhaus
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
 * Returns a string: "x", "y", or "z".
 * TO-DO: FIX THIS! This method shouldn't be needed. But, at the moment,
 * it's used as a fix to the arbitrary orientation assignment of tgBox.
 * A better solution is to have tgBox specify an orientation (ex., maybe
 * a unit vector in the "width" direction so that a set of axes can be 
 * applied to the box.) This will require changing tgBox in major ways,
 * though.
 */
std::string tgBoxMoreAnchorsInfo::getBoxOrientation() const {
  // The return value will be a string:
  std::string boxOrientation;
  // Determine box orientation by dotting the vector that runs
  // along the length of the box with (1,0,0), (0,1,0), and (0,0,1).
  btVector3 lengthVector = getTo() - getFrom();
  //DEBUGGING:
  //std::cout << "Inside tgBoxMoreAnchorsInfo::getBoxOrientation, lengthVector is "
  //	    << lengthVector << "." << std::endl;

  // For now, check to be sure the box is orthogonal to the axes of the world.
  // This class does not currently support boxes at an angle.
  // Only one of the dimensions in lengthVector should be nonzero.
  if( lengthVector.dot( btVector3(1, 0, 0)) != 0.0 ) {
    // There is a nonzero in x.
    // Check if another nonzero:
    if( lengthVector.dot( btVector3(0, 1, 0)) != 0.0 ) {
      // Box is not aligned along an axis, no prescription for such a case yet.
      throw std::invalid_argument("tgBoxMoreAnchors is not aligned along an axis! This class currently only supports boxes aligned along a single axis. Nonzeros exist in at least both x and y.");
    }
    else if( lengthVector.dot( btVector3(0, 0, 1)) != 0.0 ) {
      // Box is not aligned along an axis, no prescription for such a case yet.
      throw std::invalid_argument("tgBoxMoreAnchors is not aligned along an axis! This class currently only supports boxes aligned along a single axis. Nonzeros exist in both x and z.");
    }
    // Else, the vector only has only a nonzero in x.
    boxOrientation = "x";
  }
  else if( lengthVector.dot( btVector3(0, 1, 0)) != 0.0) {
    // There is a nonzero in y.
    // Check if also a nonzero in z:
    if( lengthVector.dot( btVector3(0, 0, 1)) != 0.0) {
      // Box is not aligned along an axis, no prescription for such a case yet.
      throw std::invalid_argument("tgBoxMoreAnchors is not aligned along an axis! This class currently only supports boxes aligned along a single axis. Nonzeros exist in both y and z.");
    }
    // Else, the vector only has a nonzero in y.
    boxOrientation = "y";
  }
  else if( lengthVector.dot( btVector3(0, 0, 1)) != 0.0) {
    // Box must be in z only.
    boxOrientation = "z";
  }
  else {
    // Must be zero length???
    throw std::runtime_error("tgBoxMoreAnchors appears to have length zero. That's not possible. Error thrown inside tgBoxMoreAnchorsInfo::getBoxOrientation.");
  }

  // Return the box's orientation.
  return boxOrientation;     
}

/**
 * The following three functions return the half extents of the box, 
 * depending on its orientation.
 * These are hard-coded by trial and error, and CHANGE ARBITRARILY.
 * Also, note that since NTRT assumes that length is the full length
 * of the box (unlike width and height, which are half-extents),
 * length has to be halved here.
 * TO-DO: we shouldn't need these functions if tgBox is changed to 
 * incorporate orientation explicitly.
 */
btVector3 tgBoxMoreAnchorsInfo::getHalfExtentsOrientedX() const {
  // Length is in X, Width is in Y, Height is in Z.
  // Used to be: width in z, height in y.
  //DEBUGGING
  //std::cout << "Length is in x direction, so width will be in y, "
  //	    << "and height will be in z. " << std::endl;
  return btVector3(getLength()/2, getConfig().width, getConfig().height);
}
btVector3 tgBoxMoreAnchorsInfo::getHalfExtentsOrientedY() const {
  // Length is in Y, Width is in X, Height is in Z.
  //DEBUGGING
  //std::cout << "Length is in y direction, so width will be in x, "
  //	      << "and height will be in z. " << std::endl;
  return btVector3(getConfig().width, getLength()/2, getConfig().height);
}
btVector3 tgBoxMoreAnchorsInfo::getHalfExtentsOrientedZ() const {
  // Length is in Z, Width is in X, Height is in Y.
  // Used to be: width in Y, height in X.
  //DEBUGGING
  //std::cout << "Length is in z direction, so width will be in X, "
  //	    << "and height will be in Y. " << std::endl;
  return btVector3(getConfig().width, getConfig().height, getLength()/2);
}

/**
 * The dimensions of the box (its half extents, in Bullet terminology) depend
 * on its orientation.
 * NOTE that width and height are used in btBoxShape in the same way that
 * radius is used for rods: they are from the center to the outside of the box,
 * NOT from one side of the box to another. 
 */
btVector3 tgBoxMoreAnchorsInfo::getHalfExtents() const {
  // The "half extents" of the box are the x, y, and z dimensions.
  btVector3 halfExtents;

  // HOWEVER, we need to figure out which dimension corresponds to the length,
  // width, and height of the box.
  // This does not seem to be consistent between boxes created in different
  // directions.
  // So, check the orientation of the box, and then get the half extents for
  // each case.
  std::string orientation = getBoxOrientation();

  // Then, call the appropriate helper function to get the half extents,
  // depending on the orientation.
  if( orientation == "x" ) {
    //DEBUGGING
    //std::cout << "Orientation is x, calling getHalfExtentsOrientedX." << std::endl;
    halfExtents = getHalfExtentsOrientedX();
  }
  else if( orientation == "y" ) {
    //DEBUGGING
    //std::cout << "Orientation is y, calling getHalfExtentsOrientedY." << std::endl;
    halfExtents = getHalfExtentsOrientedY();
  }
  else if( orientation == "z" ) {
    //DEBUGGING
    //std::cout << "Orientation is z, calling getHalfExtentsOrientedZ." << std::endl;
    halfExtents = getHalfExtentsOrientedZ();
  }
  else {
    throw std::runtime_error("Inside tgBoxMoreAnchorsInfo::getHalfExtents, orientation is neither x nor y nor z! This class currently only works with boxes aligned along one axis.");
  }

  return halfExtents;
}

/**
 * This function is used in containsNode to check if a node is on the surface
 * of the box that will be created by this tgBoxMoreAnchorsInfo.
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

  // Use the helper function to get the dimensions of the box:
  btVector3 halfExtents = getHalfExtents();

  //DEBUGGING
  if( 0 ){
    std::cout << "Box has half extents: " << halfExtents << std::endl;
  }
  
  //DEBUGGING
  if( 0 ){
    std::cout << "Checking tgBoxMoreAnchorsInfo::isNodeOnBoxSurface for box with "
	      << "pair " << getFrom() << ", " << getTo() << ", " << std::endl
	      << "...center of " << boxCenter << ", " << std::endl
	      << "...half-extents " << halfExtents << ", " << std::endl
	      << "...and node vector " << nodeVector << "." << std::endl;
    std::cout << "NOTE that this computation uses fuzzy logic: a very small number"
	      << " is added or subtracted to the half extents. " << std::endl
	      << "Currently, this number is taken from Bullet Physics' SIMD_EPSILON"
	      << " and is equal to " << tgEpsilon << "." << std::endl;
  }
  // Point is on box surface if equal to either positive or negative extents
  // in one dimension, and is within the +/- extents in the other two.
  // This is checked by comparing the location of the point with respect to
  // the box center plus or minus the half extents.
  // For now, check inclusion on surface OR inside since that's easier.
  // Version without fuzzy checking:
  /*
  bool result = (nodeVector.x() <= (boxCenter.x() + halfExtents.x())) &&
    (nodeVector.x() >= (boxCenter.x() - halfExtents.x())) &&
    (nodeVector.y() <= (boxCenter.y() + halfExtents.y())) &&
    (nodeVector.y() >= (boxCenter.y() - halfExtents.y())) &&
    (nodeVector.z() <= (boxCenter.z() + halfExtents.z())) &&
    (nodeVector.z() >= (boxCenter.z() - halfExtents.z()));
  */

  // Version WITH FUZZY CHECKING:
  bool result = (nodeVector.x() <= (boxCenter.x() + halfExtents.x() + tgEpsilon)) &&
    (nodeVector.x() >= (boxCenter.x() - halfExtents.x() - tgEpsilon)) &&
    (nodeVector.y() <= (boxCenter.y() + halfExtents.y() + tgEpsilon)) &&
    (nodeVector.y() >= (boxCenter.y() - halfExtents.y() - tgEpsilon)) &&
    (nodeVector.z() <= (boxCenter.z() + halfExtents.z() + tgEpsilon)) &&
    (nodeVector.z() >= (boxCenter.z() - halfExtents.z() - tgEpsilon));

  //DEBUGGING
  //List the result of each individual check.
  /*
  std::cout << "List of bounding box checks in each direction: " << std::endl;
  std::cout << "x_node <= x_box + halfExtents: "
	    << (nodeVector.x() <= (boxCenter.x() + halfExtents.x())) << std::endl
	    << "x_node >= x_box - halfExtents: "
	    << (nodeVector.x() >= (boxCenter.x() - halfExtents.x())) << std::endl
	    << "y_node <= y_box + halfExtents: "
	    << (nodeVector.y() <= (boxCenter.y() + halfExtents.y())) << std::endl
	    << "y_node >= y_box - halfExtents: "
	    << (nodeVector.y() >= (boxCenter.y() - halfExtents.y())) << std::endl
	    << "z_node <= z_box + halfExtents: "
	    << (nodeVector.z() <= (boxCenter.z() + halfExtents.z())) << std::endl
	    << "z_node >= z_box - halfExtents: "
	    << (nodeVector.z() >= (boxCenter.z() - halfExtents.z())) << std::endl;
  */

  //DEBUGGING:
  /*
  std::cout << "Subtracting boxCenter.x() == " << boxCenter.x() << " minus "
	    << "halfExtents.x() == " << halfExtents.x() << " equals "
	    << boxCenter.x() - halfExtents.x() << ", compare to nodeVector.x() == "
	    << nodeVector.x() << "..." << std::endl;
  */
  
  //DEBUGGING
  if( 0 ) {
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
  if( 0 ){
    std::cout << "Called containsNode inside tgBoxMoreAnchorsInfo with pair "
	      << getFrom() << ", " << getTo() << std::endl
	      << " and tags " << getTags()
	      << " for nodeVector "
	      << nodeVector << " ..." << std::endl;
  }
  // first do the older check:
  if( ((getFrom() - nodeVector).fuzzyZero() || (getTo() - nodeVector).fuzzyZero()) ){
    //DEBUGGING:
    //std::cout << "containsNode: true, is part of pair." << std::endl << std::endl;
    return true;
  }
  else {
    // Check if it's inside or on the surface of the box.
    //DEBUGGING:
    //std::cout << "Node not either of the coordinates in this box's pair. "
    //	      << "Now checking inside or on surface of box... " << std::endl;
    if( isNodeOnBoxSurface(nodeVector) ) {
      //DEBUGGING
      //std::cout << "Node is on box surface, true! " << std::endl << std::endl;
      return true;
    }

    //DEBUGGING
    //std::cout << "containsNode: false." << std::endl << std::endl;
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
