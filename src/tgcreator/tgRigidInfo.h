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

#ifndef TG_RIGID_INFO_H
#define TG_RIGID_INFO_H

/**
 * @file tgRigidInfo.h
 * @brief Definition of abstract class tgRigidInfo 
 * @author Ryan Adams
 * @date January 2014
 * $Id$
 */

// The C++ Standard Library
#include <set>
// This library
#include "core/tgTaggable.h"
#include "core/tgModel.h"
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

class btCollisionObject;
class btRigidBody;
class btCollisionShape;
class btTransform;

/**
 * A collector for keeping track of all of the necessary components of a
 * rigid model during the build process. Information is filled in in the
 * following order:
 *   1. Structural information (different for each subclass) that is 
 *      necessary for subsequent steps (e.g. radius, density, and end 
 *      points for a rod)
 *   2. m_collisionShape -- the information from #1 is used to create
 *      an appropriate btCollisionShape. Note that a btCollisionShape 
 *      does not have mass or placement info, it's just a shape.
 *   3. m_rigidGroup -- this is usually filled in by a tgStructureInfo 
 *      object during the groupRigids() operation that is required for
 *      auto-compounding. If the group is not set, a pointer to self is
 *      returned. If you know your rigid does not share any nodes with 
 *      other rigids, you can skip setting this and things will still work.
 *   4. m_rigidBody -- This is the actual representation of the rigid
 *      for Bullet Physics to use. We keep a pointer to it here so that
 *      tgConnectorInfo objects that point to this can be translated into
 *      tgModels. 
 *
 * Note: A tgRigidInfo is a tree. If it is not compound, the tree has one node.
 */ 
class tgRigidInfo : public tgTaggable {
public:
        
    tgRigidInfo() : 
        tgTaggable(),
        m_collisionShape(NULL), 
        m_rigidInfoGroup(NULL), 
        m_collisionObject(NULL)
    {}    

    tgRigidInfo(tgTags tags) : 
        tgTaggable(tags),
        m_collisionShape(NULL), 
        m_rigidInfoGroup(NULL), 
        m_collisionObject(NULL)
    {}    

    tgRigidInfo(const std::string& space_separated_tags) :
        tgTaggable(space_separated_tags),
        m_collisionShape(NULL), 
        m_rigidInfoGroup(NULL), 
        m_collisionObject(NULL)        
    {}    
    
    /** The destructor has nothing to do. */
    virtual ~tgRigidInfo() 
    {
    }
    
    // To be overridden by subclasses
    virtual tgRigidInfo* createRigidInfo(const tgNode& node)
    {
        return 0;
    }

    virtual tgRigidInfo* createRigidInfo(const tgPair& pair)
    {
        return 0;
    }

    virtual tgRigidInfo* createRigidInfo(const tgNode& node, const tgTagSearch& tagSearch);

    virtual tgRigidInfo* createRigidInfo(const tgPair& pair, const tgTagSearch& tagSearch);

    virtual std::vector<tgRigidInfo*> createRigidInfos(const tgNodes& nodes, const tgTagSearch& tagSearch);

    virtual std::vector<tgRigidInfo*> createRigidInfos(const tgPairs& pairs, const tgTagSearch& tagSearch);

    virtual void initRigidBody(tgWorld& world);

    virtual tgModel* createModel(tgWorld& world) = 0;

    /**
     * Return a pointer to the corresponding btCollisionShape.
     * @return  a pointer to the corresponding btCollisionShape
     */
    virtual btCollisionShape* getCollisionShape(tgWorld& world) const = 0;

    /**
     * Set the corresponding btCollisionShape.
     * @param[in,out] a pointer to a btCollisionShape
     */
    void setCollisionShape(btCollisionShape* p_btCollisionShape)
    {
        /// @todo Does this leak any previous value of m_collisionShape?
        m_collisionShape = p_btCollisionShape;
    }

    /**
     * Get the tgRigidInfo that represents the compound rigid
     * that this rigid belongs to. If it doesn't share nodes with
     * any other rigids (determined during auto-compounding), it
     * will point to itself.
     */
    virtual tgRigidInfo* getRigidInfoGroup()
    {
        return m_rigidInfoGroup;
    }

    virtual const tgRigidInfo* getRigidInfoGroup() const
    {
        return m_rigidInfoGroup;
    }

    /**
     * Set the tgRigidInfo that represents the compound rigid
     * that this may belong to. This will be set during auto-compounding.
     */
    virtual void setRigidInfoGroup(tgRigidInfo* rigidInfoGroup)
    {
        m_rigidInfoGroup = rigidInfoGroup;
    }

    /**
     * Return a pointer to the corresponding btRigidBody.
     * @return a pointer to the corresponding btRigidBody
     */
    virtual btRigidBody* getRigidBody();

    /**
     * Return a const pointer to the corresponding btRigidBody.
     * @return a pointer to the corresponding btRigidBody
     */
    virtual const btRigidBody* getRigidBody() const;
    
    /**
     * Set the corresponding btRigidBody.
     * @param[in,out] a pointer to a btRigidBody
     */
    virtual void setRigidBody(btRigidBody* rigidBody);
    
    /**
     * Return a pointer to the collisionObject without upcasting
     * @return a pointer to the corresponding btCollisionObject
     */
    virtual btCollisionObject* getCollisionObject()
    {
		return m_collisionObject;
	}
	
    /**
     * Return a pointer to the collisionObject without upcasting
     * @return a pointer to the corresponding btCollisionObject
     */
    virtual const btCollisionObject* getCollisionObject() const
    {
		return m_collisionObject;
	}
	
    /**
     * Set the collision object to a new collision object
     * @return a pointer to the corresponding btCollisionObject
     */
    virtual void setCollisionObject(btCollisionObject* collisionObject)
    {
		/// @todo Does this leak any previous value of m_collisionObject?
		m_collisionObject = collisionObject;
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

    /**
     * Does this rigid have any nodes in common with the given tgRigidInfo object?
     * @param]in] other a reference to a tgRigidInfo object
     * @retval true if any node in this sphere is also in other
     * @retval false if no node in this sphere is also in other
     */
    virtual bool sharesNodesWith(const tgRigidInfo& other) const;
   
    // Need these in order to see if we've already build a pair/node
    // @todo: maybe -- don't like having to know about pairs and nodes here...
    // virtual bool matches(const tgPair& pair) = 0;
    // virtual bool matches(const tgNode& node) = 0;

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
     * A pointer to the corresponding btCollisionObject.
     * Typically a btRigidBody, but can also be a btGhostObject
     */
    mutable btCollisionObject* m_collisionObject;
    
};


/**
 * Overload operator<<() to handle tgRigidInfo
 * @param[in,out] os an ostream
 * @param[in] a reference to a tgRigidInfo
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 * @todo Do we need to re-add the collision shape for debugging?
 */
inline std::ostream&
operator<<(std::ostream& os, const tgRigidInfo& obj)
{
    os << "tgRigidInfo(" << obj.getRigidInfoGroup() << ", " << obj.getRigidBody() << ")";
    return os;
}


#endif
