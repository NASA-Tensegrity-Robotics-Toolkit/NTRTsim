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

#ifndef BTEN_BOX_MORE_ANCHORS_H
#define BTEN_BOX_MORE_ANCHORS_H

/**
 * @file tgBoxMoreAnchorsInfo.h
 * @brief Class that interfaces with Bullet to build the boxes, specifically for boxes that support more than 2 anchor points.
 * @author Drew Sabelhaus
 * @date September 2016
 * $Id$
 */

// This library
#include "tgPairs.h"
//#include "tgRigidInfo.h"
#include "tgBoxInfo.h"
// The NTRT Core Library
#include "tgUtil.h"
//#include "core/tgBox.h"
#include "core/tgBoxMoreAnchors.h"

// The Bullet Physics library
#include "btBulletDynamicsCommon.h"

class btVector3;

// @todo: Need to take tags into account...

/**
 * Implementation of a box shape as defined by a 'from' point and a 'to'
 * point. It also has length, width and density.
 * This 'info' class will also detect and add nodes that are along the outside
 * edge of the box (as of 2016-08-31, actually adds nodes on or INSIDE the box,
 * which is not the behavior we want, but is good enough for the ForcePlateModel
 *
 * NOTE THAT THIS CLASS DOES NOT WORK WELL AT THE MOMENT. 
 *     A discussion on the limitations of NTRT's creation of boxes
 *     is present in tgBoxMoreAnchors.cpp, but briefly: since NTRTsim does not
 *     specify an orientation for a box, the width and height directions currently
 *     are chosen ARBITRARILY by the underlying Bullet Physics methods. As a result,
 *     this class will sometimes break for no reason. That would occur if Bullet
 *     chooses which parameter is width and which is height in a different way
 *     than is assumed here. To fix this, we'd want to change how boxes are
 *     specified, but that means changing tgBox, which may affect other code...
 */ 
class tgBoxMoreAnchorsInfo : public tgBoxInfo {
public:

    /**
     * Construct a tgBoxMoreAnchorsInfo with just a config. 
     * The pair must be filled in 
     * later, or factory methods can be used to create instances with
     * pairs.
     * note that this class is using the config from tgBox since that's easier.
     * A bit messy, but works for now.
     */
    tgBoxMoreAnchorsInfo(const tgBox::Config& config);

    /**
     * Construct a tgBoxMoreAnchorsInfo with just a config and tags. The pair must 
     * be filled in later, or factory methods can be used to create instances 
     * with pairs.
     */
    tgBoxMoreAnchorsInfo(const tgBox::Config& config, tgTags tags);

    /**
     * Construct a tgBoxMoreAnchorsInfo from its endpoints, radius and density.
     * @param[in] from one endpoint
     * @param[in] to the other endpoint
     * @param[in] config contains the radius and density
     * @todo: make sure that tgPairs returns references to the vectors...
     */
    tgBoxMoreAnchorsInfo(const tgBox::Config& config, const tgPair& pair);

    /**
     * Construct a tgBoxInfo from its endpoints, radius and density.
     * @param[in] from one endpoint
     * @param[in] to the other endpoint
     * @param[in] config contains the radius and density
     * @todo: make sure that tgPairs returns references to the vectors...
     */
    tgBoxMoreAnchorsInfo(const tgBox::Config& config, tgTags tags,
			 const tgPair& pair);
    
    /**
     * World will destroy the rigid body
     */
    virtual ~tgBoxMoreAnchorsInfo() {} 
    
    /**
     * Create a tgRigidInfo* from a tgPair.
     * Needs to be re-declared and re-defined in this class so that 
     * a tgBoxMoreAnchorsInfo will be returned, not a tgBoxInfo.
     */
    tgRigidInfo* createRigidInfo(const tgPair& pair);

    /**
     * This method needs to be re-defined to create a tgBoxMoreAnchors instead
     * of just a tgBox.
     */
    virtual tgModel* createModel(tgWorld& world);
    
    /**
     * The following three functions are part of tgBoxInfo.
     * This just makes it so that tgBoxMoreAnchorsInfo does not own the pair.
     * But that's fine. In the implementation of tgBoxMoreAnchorsInfo, we'll just
     * call these functions instead of addressing m_pair directly.
     * So, these are listed here for your reference, do not re-implement them.
     *const tgBox::Config& getConfig() const { return m_config; }
     *const btVector3& getFrom() const { return m_pair.getFrom(); }
     *const btVector3& getTo() const { return m_pair.getTo(); }
     **/
    
    /**
     * This class uses the parent's getCollisionShape method, since the underlying
     * Bullet Physics object for a box is still the same.
     * For reference, here's the line that does that creation:
     *new btBoxShape(btVector3(width, length / 2.0, height));
     */
    
    /**
     * Return a set containing only a pointer to this Box.
     * @retval a set containing only a pointer to this Box
     * @todo This function can't be const unless the return value is
     * std::set<const tgRigidInfo*>.
     * Note that this is re-defined so that a tgBoxMoreAnchorsInfo is returned
     * instead of a tgBoxInfo.
     */
    virtual std::set<tgRigidInfo*> getLeafRigids();
    
     /**
     * This function determines if a node is on the surface (*ON THE INSIDE*)
     * of this box. This is the big difference from tgBoxInfo! In that class,
     * this function just checked if the nodeVector was either of the two endpoints.
     */
    virtual bool containsNode(const btVector3& nodeVector) const;

    /**
     * Return a set containing all the nodes in this box. Note that
     * tgBoxInfo has this same function, and we're redefining it here
     * so that the returned set will include the additional nodes after they're 
     * added to additionalNodes by the containsNode method.
     * @TODO: actually implement this. right now, just use the parent's function.
     */
    //    virtual std::set<btVector3> getContainedNodes() const;

protected:

    /**
     * This function checks if a node is on the surface of the tgBox that will be
     * created with this tgBoxInfo. 
     * NOTE that it does NOT work for boxes that start off rotated:
     * unless a box is orthogonal to the axes of the world, 
     * this function will always return false.
     * It is protected so that only children of this class could call or re-define.
     * @param[in] nodeVector a btVector3 that is to be checked against this box.
     * @retval result, true if the node is on or inside the box, false else.
     */
    bool isNodeOnBoxSurface(const btVector3& nodeVector) const;

    /**
     * This function is a helper to isNodeOnBoxSurface.
     * It returns the correct geometry of the box around which to check for nodes.
     * This is needed because it's unclear what direction is width and which
     * is height for a box. We fix this by testing and hard-coding the width/height
     * ordering for different orientations of the box.
     * This is done because the Bullet Physics object is not created yet:
     * containsNode is called before the methods that create the Bullet object.
     * @TODO: can this program determine what orientation and rotations will
     * be used by Bullet, and do that instead of hard-coding?
     * @retval a btVector3 of half extents, just like Bullet uses for btBoxShape.
     */
    btVector3 getHalfExtents() const;

    /**
     * This function is a helper inside getHalfExtents.
     * It returns the orientation of the box.
     * @retval a string with the axis along which this box exists.
     * This should be enough to determine how to orient the half extents.
     */
    std::string getBoxOrientation() const;

    /**
     * The following three functions are helpers to getHalfExtents.
     * They each assign the half extents according to the specific orientation
     * of the box.
     */
    btVector3 getHalfExtentsOrientedX() const;
    btVector3 getHalfExtentsOrientedY() const;
    btVector3 getHalfExtentsOrientedZ() const;

private:

    /** Disable the copy constructor. */
    tgBoxMoreAnchorsInfo(const tgBoxMoreAnchorsInfo&);

    /** Disable the assignment operator. */
    tgBoxMoreAnchorsInfo& operator=(const tgBoxMoreAnchorsInfo&);

    // The following variable is used in the half extents checking.
    // This "SIMD_EPSILON" is what btVector3 uses in its fuzzyZero method,
    // which motivates its use here.
    // @TODO: will this not work sometimes??
    double tgEpsilon = SIMD_EPSILON * 10;

    // note that tgBoxInfo has the following two private variables:
    //const tgPair m_pair;
    //const tgBox::Config m_config;     

};

/**
 * Overload operator<<() to handle a tgBoxMoreAnchorsInfo.
 * @param[in,out] os an ostream
 * @param[in] q a tgBoxMoreAnchorsInfo
 * @return os
 */
inline std::ostream& operator<<(std::ostream& os, const tgBoxMoreAnchorsInfo& box)
{
    os << "tgBoxMoreAnchorsInfo(" << box.getFrom() << ", " << box.getTo() <<")";
    return os;
}

#endif
