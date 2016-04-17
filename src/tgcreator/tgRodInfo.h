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

#ifndef BTEN_ROD_H
#define BTEN_ROD_H

/**
 * @file tgRodInfo.h
 * @brief Definition of class tgRodInfo 
 * @author Ryan Adams
 * @date January 2014
 * $Id$
 */

// This library
#include "tgPairs.h"
#include "tgRigidInfo.h"
// The NTRT Core Library
#include "tgUtil.h"
#include "core/tgRod.h"

// The Bullet Physics library
#include "btBulletDynamicsCommon.h"

class btVector3;

// @todo: Need to take tags into account...

/**
 * Implementation of a cylinder shape as defined by a 'from' point and a 'to'
 * point. It also has radius and density.
 */ 
class tgRodInfo : public tgRigidInfo {
public:

    /**
     * Construct a tgRodInfo with just a config. The pair must be filled in 
     * later, or factory methods can be used to create instances with
     * pairs.
     */
    tgRodInfo(const tgRod::Config& config);

    /**
     * Construct a tgRodInfo with just a config and tags. The pair must 
     * be filled in later, or factory methods can be used to create instances 
     * with pairs.
     */
    tgRodInfo(const tgRod::Config& config, tgTags tags);

    /**
     * Construct a tgRodInfo from its endpoints, radius and density.
     * @param[in] from one endpoint
     * @param[in] to the other endpoint
     * @param[in] config contains the radius and density
     * @todo: make sure that tgPairs returns references to the vectors...
     */
    tgRodInfo(const tgRod::Config& config, const tgPair& pair);

    /**
     * Construct a tgRodInfo from its endpoints, radius and density.
     * @param[in] from one endpoint
     * @param[in] to the other endpoint
     * @param[in] config contains the radius and density
     * @todo: make sure that tgPairs returns references to the vectors...
     */
    tgRodInfo(const tgRod::Config& config, tgTags tags, const tgPair& pair);
    
    /**
     * World will destroy the rigid body
     */
    virtual ~tgRodInfo() {} 
    
    /**
     * Create a tgRigidInfo* from a tgPair
     */ 
    tgRigidInfo* createRigidInfo(const tgPair& pair);
    
    /**
     * Call tgRigidInfo init rigid, then apply config to the rigid body.
     * @todo come up with a general solution in tgRigidInfo::initRigidBody
     * Currently very difficult to pass around the config file in
     * tgRigidInfo, since
     */
    virtual void initRigidBody(tgWorld& world);
    
    tgModel* createModel(tgWorld& world);
    
    /**
     * Return a const reference to the container of the radius and density.
     * @return a const reference to the container of the radius and density
     */
    const tgRod::Config& getConfig() const { return m_config; }

    /** Return a const reference to the first endpoint. */
    const btVector3& getFrom() const { return m_pair.getFrom(); }
    
    /** Return a const reference to the second endpoint. */
    const btVector3& getTo() const { return m_pair.getTo(); }
    
    /**
     * Return a pointer to the corresponding btCollisionShape, lazily creating
     * it if it does not exist.
     */
    virtual btCollisionShape* getCollisionShape(tgWorld& world) const;
    
    /**
     * Return a btTransform that maps the from endpoint to the to endpoint
     * @return a btTransform that maps the from endpoint to the to endpoint
     */
    virtual btTransform getTransform() const
    {
        return tgUtil::getTransform(getFrom(), getTo());
    }
    
    /**
     * Return the rod's mass.
     * The mass is the volume times the density.
     * @return the mass of the rod
     */
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

    /**
     * Return the distance between the two endpoints.
     * @return the distance between the two endpoints
     */
    double getLength() const {
        return getFrom().distance(getTo());
    }
  
private:

    /** Disable the copy constructor. */
    tgRodInfo(const tgRodInfo&);

    /** Disable the assignment operator. */
    tgRodInfo& operator=(const tgRodInfo&);  
    
private:

    /** The pair representing the endpoints of the rod. */
    const tgPair m_pair;
    
    /** Radius and density values. */
    const tgRod::Config m_config;

};

/**
 * Overload operator<<() to handle a tgRodInfo.
 * @param[in,out] os an ostream
 * @param[in] q a tgRodInfo
 * @return os
 */
inline std::ostream& operator<<(std::ostream& os, const tgRodInfo& rod)
{
    os << "tgRodInfo(" << rod.getFrom() << ", " << rod.getTo() <<")";
    return os;
}

#endif
