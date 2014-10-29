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

#ifndef TG_GHOST_INFO_H
#define TG_GHOST_INFO_H

/**
 * @file tgGhostInfo.h
 * @brief Definition of abstract class tgGhostInfo 
 * @author Brian Mirletz
 * @date October 2014
 * $Id$
 */

// The C++ Standard Library
#include <set>
// This library
#include "core/tgTaggable.h"
#include "core/tgModel.h"
#include "tgcreator/tgRigidInfo.h"
//Bullet Physics
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"

// Forward references
class tgCompoundRigidInfo;
class tgNode;
class tgNodes;
class tgPair;
class tgPairs;
class tgTagSearch;
class tgWorld;

class btRigidBody;
class btCollisionShape;
class btTransform;

/**
 * Builds off of tgRigidInfo, but creates ghost objects instead of
 * rigid bodies. Needs to be in its own seperate structure, so
 * these don't get accidentally compounded with rigid objects.
 * Used by MuscleNP and tgMultiPointStringInfo
 */ 
class tgGhostInfo : public tgTaggable {
public:
        
    tgGhostInfo() : 
        tgRigidInfo()
    {}    

    tgGhostInfo(tgTags tags) : 
        tgRigidInfo(tags)
    {}    

    /** The destructor has nothing to do. */
    virtual ~tgGhostInfo() 
    {
    }
    
    // To be overridden by subclasses
    virtual tgGhostInfo* createRigidInfo(const tgNode& node)
    {
        return 0;
    }

    virtual tgGhostInfo* createRigidInfo(const tgPair& pair)
    {
        return 0;
    }
	
    virtual void initRigidBody(tgWorld& world);

    virtual tgModel* createModel(tgWorld& world) = 0;

    /**
     * Return a pointer to the corresponding btCollisionShape.
     * @return  a pointer to the corresponding btCollisionShape
     */
    virtual btCollisionShape* getCollisionShape(tgWorld& world) const = 0;
    
    /**
     * Set the corresponding btRigidBody.
     * @param[in,out] a pointer to a btRigidBody
     */
    virtual void setRigidBody(btRigidBody* rigidBody)
    {
        /// @todo Does this leak any previous value of m_rigidBody?
        assert(
    }
        
    /**
     * Return a btTransform.
     * @return a btTransform
     */
    virtual btTransform getTransform() const = 0;
    
    /**
     * Return the rigid body's mass.
     * @return the rigid bddy's mass
     */
    virtual double getMass() const = 0;
    
    /**
     * Return the rigid body's center of mass.
     * @return the rigid body's center of mass.
     */
    virtual btVector3 getCenterOfMass() const = 0;

    /**
     * Add this (for determining, for instance, an edge connection point for a
     * cylinder, a surface point on a ball, etc.)
     * Usually the referencePoint is something like the node at the end of a
     * cylinder or the center point of a sphere, and the destinationPoint is a
     * point on the other object that we'll be connecting to. In the case of a
     * cylinder, this function might return the edge point closest to the
     * destinationPoint.
     *
     * Or for an irregular surface (or any shape for that matter), you could
     * keep a list of surface points and have
     * this function select the closest one to the destination. 
     * 
     * @todo: For the generic tgRigidInfo implementation, allow people to provide
     * a delegate object to find the connection point.
     * @todo: Should this return a reference instead of a value? Possibility of
     * change indicates a reference, but do we want to allow this to be
     * changeable?
    */
    virtual btVector3 
    getConnectionPoint(const btVector3& referencePoint,
               const btVector3& destinationPoint) const = 0;
    
    /**
     * Virtual version of getConnectionPoint quietly ignores rotation
     * information for non-rod objects
     */        
    virtual btVector3 
    getConnectionPoint(const btVector3& referencePoint,
               const btVector3& destinationPoint,
               const double rotation) const
   {
       return getConnectionPoint(referencePoint, destinationPoint);
   }

    /**
     * Is this tgRigidInfo a compound?
     * @retval true if this tgRigidInfo is a compound
     * @retval false if this tgRigidInfo is not a compound
     * @todo Is this necessary?
     */
    bool isCompound() const { return getCompound() != 0; }
        
    /**
     * If this tgRigidInfo is a tgCompoundRigidInfo, return a pointer to it; otherwise
     * return NULL.
     * @retval NULL if this tgRigidInfo is not a tgCompoundRigidInfo
     * @retval the this pointer if this tgRigidInfo is a tgCompoundRigidInfo
     * @todo Is this necessary?
     */
    virtual tgCompoundRigidInfo* getCompound() = 0;
        
    /**
     * If this tgRigidInfo is a tgCompoundRigidInfo, return a pointer to it; otherwise
     * return NULL.
     * @retval NULL if this tgRigidInfo is not a tgCompoundRigidInfo
     * @retval the this pointer if this tgRigidInfo is a tgCompoundRigidInfo
     * @todo Is this necessary?
     */
    virtual const tgCompoundRigidInfo* getCompound() const = 0;

    /**
     * Return a set of all non-compound tgRigidInfo objects in the tgRigidInfo tree.
     * @todo This function can't be const unless the return value is
     * std::set<const tgRigidInfo*>.
     */
    virtual std::set<tgRigidInfo*> getLeafRigids() = 0;

    /**
     * Is the given vector a node anywhere in this rigid body?
     * @retval true if nodeVector is a node anywhere in this rigid body
     * @retval false if nodeVector is not a node anywhere in this rigid body
     */
    virtual bool containsNode(const btVector3& nodeVector) const = 0;

    /**
     * Return the set of nodes contained anywhere in the rigid body. 
     * For instance, a tgRod would return a set containing its 'from' and
     * 'to' points.
     */
    virtual std::set<btVector3> getContainedNodes() const = 0;

	

protected:  // Protected, not private -- subclasses need access
    
    /**
     * A pointer to the corresponding btCollisionShape.
     */
    mutable btCollisionShape* m_collisionShape;

    /**
     * A pointer to a group of rigids to which this 
     * rigid belongs. This is set during the rigid 
     * grouping step by a StructureInfo. If this is
     * not set, getRigidGroup() will return a pointer
     * to this (effectively, "I'm in my own group")
     */
    mutable tgRigidInfo* m_rigidInfoGroup;

    /**
     * A pointer to the corresponding btRigidBody.
     */
    mutable btRigidBody* m_rigidBody;
    
};



#endif
