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

#ifndef NTRT_MUSCLENP_H_
#define NTRT_MUSCLENP_H_

/**
 * @file MuscleNP.h
 * @brief Definition of a massless cable with contact dynamics
 * @author Brian Mirletz
 * @date November 2014
 * $Id$
 */

// NTRT
#include "core/Muscle2P.h"
// The Bullet Physics library
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
// The C++ Standard Library

#include <vector>

// Forward references
class tgWorld;
class tgBulletSpringCableAnchor;
class btRigidBody;
class btCollisionShape;
class btCompoundShape;
class btPairCachingGhostObject;
class btDynamicsWorld;

/**
 * An extension of Muscle2P that places a ghostObject into the bullet world
 * and then uses that object to generate collision dynamics. The
 * ghost object is updated every time step to account for the new
 * shape of the string. Anchors track their associated btPersistentManifold
 * to determine whether they are still in contact with the rigid object. The string
 * is massless, but collision handling should conserve momentum (see unit tests)
 * 
 * This could eventually be merged with Corde for a massive, low node
 * string. Merging with Corde could also address some of the current
 * issues with rotational energy
 */
class MuscleNP : public Muscle2P
{
public:
	
	/**
	 * The only constructor. Requires a number of parameters typically
	 * provided by the builder tools, in this case tgMultiPointStringInfo
	 * @param[in] ghostObject - the btPairCashingGhostObject that this interacts with
	 * @param[in] world - the tgWorld - we need to keep access to the bullet 
	 * dynamics world's broadphase so that we can determine collisions and contact points
	 * @param[in] anchors - a reference to a list of anchors. Will be passed to Muscle2P
	 * and form the initial (likely permanent) anchor list. Size must be 2 or greater
	 * (enforced in Muscle2P)
	 * @param[in] coefK - The stiffness of this cable. Units are 
	 * mass / sec^2, sets a member variable of Muscle2P
	 * @param[in] dampingCofefficient - The damping coefficient of the force application equation.
	 * Units are mass / sec, sets a member variable of Muscle2P
	 * @param[in] The pretension on the string, units are force. Must be small enough 
	 * (based on stiffness) such that rest length > 0;
	 * @param[in] thickness, the radius of the cylinder used for the btCollisionObject
	 * @param[in] resolution, the spatial resultion used to prune new contacts. 
	 * also affects runtime (lower corresponds to longer runtime)
	 */
    MuscleNP(btPairCachingGhostObject* ghostObject,
				tgWorld& world,
				const std::vector<tgBulletSpringCableAnchor*>& anchors,
				double coefK,
				double dampingCoefficient,
				double pretension = 0.0,
				double thickness = 0.001,
				double resolution = 0.1);
    /**
     * The destructor. Removes the ghost object from the world,
     * deletes its collision shape, and then deletes the object.
     * Muscle2P ensures all anchors are deleted
     */     
	virtual ~MuscleNP();
    
    /**
     * @return a btScalar of the string's actual length - the sum of the
     * lengths between the anchors.
     */
    virtual const btScalar getActualLength() const;
    
    /**
     * The "update" function for MuscleNP. In addition to calculating and
     * applying the force, this calls:
     * updateManifolds()
     * pruneAnchors()
     * updateAnchorList()
     * pruneAnchors()
     * THEN it calculates and applies the forces (calculating permanent anchors
     * and sliding contacts slightly differently) and distributes them so momentum
     * is conserved
     * finally updateCollisionObject()
     * @todo change this to update(dt) or similar, since that's the role its serving
    */
    virtual void calculateAndApplyForce(double dt);
    
private:
    
    /**
     * Function object to compare the spatial position of two anchors,
     * given two other anchors of interest. Used to place new anchors
     */
    struct anchorCompare
    {
        anchorCompare(const tgBulletSpringCableAnchor* m1, const tgBulletSpringCableAnchor* m2);
        
        /**
         * Calls comparePoints on anchors lhs and rhs
         */
        bool operator() (const tgBulletSpringCableAnchor* lhs, const tgBulletSpringCableAnchor* rhs) const;
		
		/**
		 * Sees which of the points is further along the line between ma1
		 * and ma2
		 */
        bool comparePoints(btVector3& pt2, btVector3& pt3) const;
        
        private:
           const tgBulletSpringCableAnchor* const ma1;
           const tgBulletSpringCableAnchor* const ma2;
    };
    
    /**
     * Iterate through the pairs of objects to find the contact positions
     * Contacts are then compared with existing anchors, and either the
     * btPersistantManifold of the anchors is updated, or the point is
     * stored in m_newAnchors as a new anchor
     */
    void updateManifolds();
    
    /**
     * Iterates through the list of new anchors created by updateManifolds
     * and checks whether new anchors are in the same position as
     * existing anchors, or if they would 'push' against the colliding
     * object. Valid anchors are added to m_anchors.
     */
    void updateAnchorList();
    
    /**
     * First minimizes the length of the string in allowed directions
     * and deletes anchors with out of date manifolds. Then deletes
     * anchors that are too close to surrounding anchors or where
     * normals would push.
     */
    void pruneAnchors();
    
    /**
     * Uses m_anchors to update the collision shape of the m_ghostObject
     * Also resets the broadphase's pairCache after collision object
     * is changed.
     */
    void updateCollisionObject();
    
    /**
     * Deletes a collision shape and it's child shapes
     * @param[in] pShape the btCollisionShape to be deleted
     */
    void deleteCollisionShape(btCollisionShape* pShape);
    
    /**
     * Deletes the child shapes of the collision object, leaving
     * an empty container
     * @param[in] pShape the shape to be emptied
     */
    void clearCompoundShape(btCompoundShape* pShape);
    
    /**
     * Determine if the anchor at i is permanent, if not, delete it
     * and remove it from m_anchors.
     * @param[in] i the index of the anchor to be deleted
     * @return true if the anchor has been deleted
     */
    bool deleteAnchor(int i);
    
    /**
     * Find the anchor closest to this position in space, then return
     * the index of the anchor that would immediately proceed it
     * once it is inserted into m_anchors. Used by both updateManifolds()
     * and updateAnchorList()
     * @param[in] the position of the contact or anchor in question
     * @return the index of the relevant anchor
     * @todo Introduce more flexibility to this function for contacts
     * between the two anchors that are not along a line 
     */
    int findNearestPastAnchor(btVector3& pos);
    
    /**
     * An iterator over a list of tgBulletSpringCableAnchors. Used to insert new
     * anchors during updateAnchorList()
     */
    std::vector<tgBulletSpringCableAnchor*>::iterator m_anchorIt;
   
    /**
     * The total force experienced by the sliding anchors
     */
    btVector3 m_forceTotals;
    
    /**
     * The scaling factor between the sliding and permanent anchors that
     * (usually) ensures momentum is conserved
     */
    btVector3 m_forceScales;
    
    /**
     * Temporary storage for anchors between updateManifolds() and
     * updateAnchorList()
     */
    std::vector<tgBulletSpringCableAnchor*> m_newAnchors;
    
    /**
     * A reference to the dynamics world so that we can track the
     * contact points in the broadphase's pairCache and remove
     * the ghostObject from the collision world
     */
    tgWorld&  m_world;
    
protected:  
    
    /**
     * A btPairCachingGhostObject that exists in the dynamics world
     * and provides us information about what it collides with.
     * We own this.
     */
    btPairCachingGhostObject* m_ghostObject;
	
	/**
	 * The radius of the ghost object
	 * Units of length
	 */
	const double m_thickness;
	
	/**
	 * The spatial resolution used in collision detection
	 * Units of length
	 */
	const double m_resolution;
};

#endif  // NTRT_MUSCLENP_H_
