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

#include <string>
#include <vector>
#include <map>

// Forward references
class tgWorld;
class muscleAnchor;
class btRigidBody;
class btCollisionShape;
class btCompoundShape;
class btPairCachingGhostObject;
class btDynamicsWorld;
class btBroadphaseInterface;
class btPersistentManifold;
class btDispatcher;

/**
 * An extension of Muscle2P that places a ghostObject into the bullet world
 * and then uses that object to generate collision dynamics. The
 * ghost object is updated every time step to account for the new
 * shape of the string. Anchors track their associated btPersistentManifold
 * to determine whether they are still in contact with the rigid object. The string
 * is massless, but collision handling should conserve momentum (see unit tests)
 * 
 * This could eventually be merged with Corde for a massive, low node
 * string.
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
	 * @param[in] body1 - The rigid body associated with anchor1, a member variable of Muscle2P
	 * @param[in] pos1 - The world position associated with anchor1, a member variable of Muscle2P
	 * @param[in] body2 - The rigid body associated with anchor2, a member variable of Muscle2P
	 * @param[in] coefK - The stiffness of this cable. Units are 
	 * mass / sec^2, sets a member variable of Muscle2P
	 * @param[in] dampingCofefficient - The damping coefficient of the force application equation.
	 * Units are mass / sec, sets a member variable of Muscle2P
	 */
    MuscleNP(btPairCachingGhostObject* ghostObject,
            tgWorld& world,
         btRigidBody * body1,
         btVector3 pos1,
         btRigidBody * body2,
         btVector3 pos2,
         double coefK,
         double dampingCoefficient);
    
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
  
    struct anchorCompare
    {
        anchorCompare(const muscleAnchor* m1, const muscleAnchor* m2);
        
        bool operator() (const muscleAnchor* lhs, const muscleAnchor* rhs) const;
    
        bool comparePoints(btVector3& pt2, btVector3& pt3) const;
        
        private:
           const muscleAnchor* const ma1;
           const muscleAnchor* const ma2;
    };
    
    void updateManifolds();
    
    void updateAnchorList();
    
    void pruneAnchors();
    
    void updateCollisionObject();
    
    void deleteCollisionShape(btCollisionShape* pShape);
    
    void clearCompoundShape(btCompoundShape* pShape);
    
    // Returns whether the anchor was deleteable
    bool deleteAnchor(int i);
    
    int findNearestPastAnchor(btVector3& pos);
    
    std::vector<muscleAnchor*>::iterator m_anchorIt;
   
    btVector3 m_forceTotals;
    btVector3 m_forceScales;
   
    std::vector<muscleAnchor*> m_newAnchors;
    
    tgWorld&  m_world;
    
protected:  
    
   btPairCachingGhostObject* m_ghostObject;

};

#endif  // NTRT_MUSCLENP_H_
