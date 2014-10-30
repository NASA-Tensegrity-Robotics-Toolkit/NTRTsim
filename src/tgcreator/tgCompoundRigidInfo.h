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

#ifndef TG_COMPOUND_RIGID_INFO_H
#define TG_COMPOUND_RIGID_INFO_H

/**
 * @file tgCompoundRigidInfo.h
 * @brief Definition of class tgCompoundRigidInfo 
 * @author Ryan Adams
 * @date January 2014
 * $Id$
 */

#include "tgRigidInfo.h"
#include "core/tgWorldBulletPhysicsImpl.h"
// @todo: do we just need the btCompoundShape here?
#include "btBulletDynamicsCommon.h"
#include <vector>

class tgCompoundRigidInfo : public tgRigidInfo 
{
 public:

    /**
     * The null constructor is the default constructor.
     * @todo Require both m_rigidBody and m_rigids to be supplied in the
     * constructor. Initialize m_compoundShape, store mass and center of mass
     * in member variables.
     */
    tgCompoundRigidInfo();

    tgModel* createModel(tgWorld& world);

    /**
     * The destructor does not own the btRigidBody objects and must not
     * deallocate them.
     * Additionally, collision shapes cannot be deallocated until the
     * world is deleted
     */
    virtual ~tgCompoundRigidInfo()
    {
    }

    /**
     * Insert a pointer to the supplied tgRigidInfo into a container.
     * The tgCompoundRigidInfo does not assume ownership of the tgRigidInfo, and must not
     * deallocate it.
     * @param[in,out] rigit a tgRigidInfo
     * @todo Get rid of this. Require all m_rigids to be supplied in the
     * constructor.
     */
    void addRigid(tgRigidInfo& rigid);
    
    /**
     * Return the center of mass of all the tgRigidInfo objects in the tree.
     * @return the center of mass of all the tgRigidInfo objects in the tree
     * @retval a zero vector if the mass is zero
     * @todo Make this const here and in all base classes and derived classes.
     * @todo Make rigid const when tgCompoundRigidInfo::getCenterOfMass() is const.
     * @todo If all m_rigids are supplied in the constructor, this can be
     * calculated in the constructor and cashed as a const member variable.
     */
    virtual btVector3 getCenterOfMass() const;
    
    /**
     * Return a pointer to the corresponding btCollisionShape, lazily creating
     * it if it does not exist..
     * @return  a pointer to the corresponding btCollisionShape
     */
    btCompoundShape* createCompoundShape(tgWorld& world) const;

    /**
     * Return a pointer to the corresponding btCollisionShape.
     * @return  a pointer to the corresponding btCollisionShape
     */
    virtual btCollisionShape* getCollisionShape(tgWorld& world) const;

    /**
     * Return an identity btTransform with the origin being the center of mass.
     * @return an identity btTransform with the origin being the center of mass
     */
    virtual btTransform getTransform() const;
    
    /**
     * Return the compound's mass.
     * The mass is the sum of the masses of all the tgRigidInfo objects in the
     * compound.
     * @todo Do this in the constructor and cache the result as a member
     * variable.
     */
    virtual double getMass() const;

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
    virtual void setRigidBody(btRigidBody* const rigidBody);
    
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
    virtual void setCollisionObject(btCollisionObject* collisionObject);
      
    /**
     * By default this should look for the object that has the closest
     * connectionPoint to the destination or something. 
     * That may change, or we might provide a strategy-based object here?
     */
    virtual btVector3
    getConnectionPoint(const btVector3& referencePoint,
               const btVector3& /* destinationPoint */) const
    {
        // @todo: make this work...
        return referencePoint;
    }
    
    /**
     * Return a non-const pointer to this object.
     * @todo What is the purpose? The caller must already have the object,
     * a reference or a pointer to be able to call this.
     */
    virtual tgCompoundRigidInfo * getCompound() { return this; }
    
    /**
     * Return a const pointer to this object.
     * @todo What is the purpose? The caller must already have the object,
     * a reference or a pointer to be able to call this.
     * @todo Add this to the base classes and derived classes.
     */
    virtual const tgCompoundRigidInfo * getCompound() const { return this; }
    
    /**
     * @todo This function can't be const unless the return value is
     * std::set<const tgRigidInfo*>.
     */
    virtual std::set<tgRigidInfo*> getLeafRigids();
    
    /**
     * Is the given vector a node anywhere in this compound?
     * @retval true if nodeVector is a node anywhere in this compound
     * @retval false if nodeVector is not a node anywhere in this compound
     */
    virtual bool containsNode(const btVector3& nodeVector) const;
    
    /**
     * @todo Make this const in all base classes and all derived classes.
     * @todo Make other const in all base classes and all derived classes.
     */
    virtual bool sharesNodesWith(const tgRigidInfo& other) const;
    
    /**
     * Return a set of the nodes contained anywhere in this compound.
     * @return a set of the nodes contained anywhere in this compound
     */
    std::set<btVector3> getContainedNodes() const;

protected:

    /**
     * A collection of tgRigidInfo pointers, each supplied by the client.
     * @todo Change this to std::set to prevent duplication.
     * @todo Make this const and initialize it in the constructor.
     */
    /* const */ std::vector<tgRigidInfo*> m_rigids;

    /**
     * The btCompoundShape that represents this compound to Bullet.
     */
    mutable btCompoundShape * m_compoundShape;

};


#endif
