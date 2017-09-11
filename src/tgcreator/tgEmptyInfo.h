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

#ifndef tgEmptyInfo_H
#define tgEmptyInfo_H

// This library
// The NTRT Core Library
#include "tgUtil.h"
#include "tgPairs.h"
#include "tgRigidInfo.h"

// The Bullet Physics library
#include "btBulletDynamicsCommon.h"

class tgEmptyInfo : public tgRigidInfo {
public:
    tgEmptyInfo();
    tgEmptyInfo(const tgPair& pair);
    
    virtual ~tgEmptyInfo() {} 
    
    tgEmptyInfo* createRigidInfo(const tgPair& pair);
    virtual void initRigidBody(tgWorld& world);
    
    tgModel* createModel(tgWorld& world);
   
    virtual btCollisionShape* getCollisionShape(tgWorld& world) const;
    
    /** Return a const reference to the first endpoint. */
    const btVector3& getFrom() const { return m_pair.getFrom(); }
    
    /** Return a const reference to the second endpoint. */
    const btVector3& getTo() const { return m_pair.getTo(); }
    

    virtual btTransform getTransform() const
    {
        return tgUtil::getTransform(getFrom(), getTo());
    }
    

    virtual double getMass() const;
    /**
     * Return the rod's center of mass.
     * The center of mass is a point halfway between the endpoints.
     * @return the rod's center of mass
     */
    virtual btVector3 getCenterOfMass() const
    {
        return (getFrom() + getTo()) / 2.0;
    }
    
    /**
     * Return the surface point closest to the reference point in the
     * direction of the destination point.
     * @param[in] referencePoint a btVector
     * @param[in] destinationPoint a btVector
     * @return the surface point closest to the reference point in the
     * direction of the destination point
     */
    virtual btVector3 
    getConnectionPoint(const btVector3& referencePoint,
                       const btVector3& destinationPoint) const;
    
    /**
     * Return the surface point closest to the reference point in the
     * direction of the destination point.
     * @param[in] referencePoint a btVector
     * @param[in] destinationPoint a btVector
     * @return the surface point closest to the reference point in the
     * direction of the destination point
     */
    virtual btVector3 
    getConnectionPoint(const btVector3& referencePoint,
                       const btVector3& destinationPoint,
                       const double rotation) const;
    
    /**
     * Since a rod is not a compound shape, there is no compound shape object to
     * return.
     * @retval NULL
     */
    virtual tgCompoundRigidInfo* getCompound() { return 0; }

    /**
     * Since a rod is not a compound shape, there is no compound shape object to
     * return.
     * @retval NULL
     */
    virtual const tgCompoundRigidInfo* getCompound() const { return 0; }

    /**
     * Return a set containing only a pointer to this rod.
     * @retval a set containing only a pointer to this rod
     * @todo This function can't be const unless the return value is
     * std::set<const tgRigidInfo*>.
     */
    virtual std::set<tgRigidInfo*> getLeafRigids();
    
     /**
     * Is the given vector either of the endpoints?
     * @retval true if nodeVector is either endpoint
     * @retval false if nodeVector is neither endpoint
     */
    virtual bool containsNode(const btVector3& nodeVector) const 
    {
        return ((getFrom() - nodeVector).fuzzyZero() || (getTo() - nodeVector).fuzzyZero());
    }

    /**
     * Return a set contiaining the two endpoints.
     * @return a set contiaining the two endpoints
     */
    virtual std::set<btVector3> getContainedNodes() const;

private:
/** The pair representing the endpoints of the rod. */
    const tgPair m_pair;
};

#endif
