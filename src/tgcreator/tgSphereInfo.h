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

#ifndef BTEN_SPHERE_H
#define BTEN_SPHERE_H

/**
 * @file tgSphereInfo.h
 * @brief Definition of class tgSphereInfo 
 * @author Ryan Adams
 * @date January 2014
 * $Id$
 */

// This library
#include "tgNode.h"
#include "tgRigidInfo.h"
// The NTRT Core Library
#include "tgUtil.h"
#include "core/tgSphere.h"

// The Bullet Physics library
#include "btBulletDynamicsCommon.h"

class btVector3;

// @todo: Need to take tags into account...

/**
 * Implementation of a cylinder shape as defined by a 'from' point and a 'to'
 * point. It also has radius and density.
 */ 
class tgSphereInfo : public tgRigidInfo {
public:

    /**
     * Construct a tgSphereInfo with just a config. The node must be filled in 
     * later, or factory methods can be used to create instances with
     * nodes.
     */
    tgSphereInfo(const tgSphere::Config& config);

    /**
     * Construct a tgSphereInfo with just a config and tags. The node must 
     * be filled in later, or factory methods can be used to create instances 
     * with nodes.
     */
    tgSphereInfo(const tgSphere::Config& config, tgTags tags);

    /**
     * Construct a tgSphereInfo from its center point, radius and density.
     * @param[in] the center point
     * @param[in] config contains the radius and density
     * @todo: make sure that tgPairs returns references to the vectors...
     */
    tgSphereInfo(const tgSphere::Config& config, const tgNode& node);

    /**
     * Construct a tgSphereInfo from its center point, radius and density.
     * @param[in] the center point
     * @param[in] config contains the radius and density
     * @todo: make sure that tgPairs returns references to the vectors...
     */
    tgSphereInfo(const tgSphere::Config& config, tgTags tags, const tgNode& node);
    
    /**
     * World will destroy the rigid body
     */
    virtual ~tgSphereInfo() {} 
    
    /**
     * Create a tgRigidInfo* from a tgPair
     */ 
    tgRigidInfo* createRigidInfo(const tgNode& node);
    
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
    const tgSphere::Config& getConfig() const { return m_config; }

    /** 
     * Return a const reference to the center.
     * Basically just casting to btVector3
     */
    const btVector3& getNode() const { return m_node; }
    
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
        return tgUtil::getTransform(getNode());
    }
    
    /**
     * Return the sphere's mass.
     * The mass is the volume times the density.
     * @return the mass of the sphere
     */
    virtual double getMass() const;
    /**
     * Return the sphere's center of mass.
     * The center of mass is a point halfway between the endpoints.
     * @return the sphere's center of mass
     */
    virtual btVector3 getCenterOfMass() const
    {
        return getNode();
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
     * @todo support rotation without a proper normal vector (spherical coords?)
     */
    virtual btVector3 
    getConnectionPoint(const btVector3& referencePoint,
                       const btVector3& destinationPoint,
                       const double rotation) const;
    
    /**
     * Since a sphere is not a compound shape, there is no compound shape object to
     * return.
     * @retval NULL
     */
    virtual tgCompoundRigidInfo* getCompound() { return 0; }

    /**
     * Since a sphere is not a compound shape, there is no compound shape object to
     * return.
     * @retval NULL
     */
    virtual const tgCompoundRigidInfo* getCompound() const { return 0; }

    /**
     * Return a set containing only a pointer to this sphere.
     * @retval a set containing only a pointer to this sphere
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
        return (getNode() == nodeVector);
    }

    /**
     * Return a set contiaining the center point twice
     * @return a set contiaining the center poin
     */
    virtual std::set<btVector3> getContainedNodes() const;

private:

    /** Disable the copy constructor. */
    tgSphereInfo(const tgSphereInfo&);

    /** Disable the assignment operator. */
    tgSphereInfo& operator=(const tgSphereInfo&);  
    
private:

    /** The node representing the center of the sphere. */
    const tgNode m_node;
    
    /** Radius and density values. */
    const tgSphere::Config m_config;     

};

/**
 * Overload operator<<() to handle a tgSphereInfo.
 * @param[in,out] os an ostream
 * @param[in] q a tgSphereInfo
 * @return os
 */
inline std::ostream& operator<<(std::ostream& os, const tgSphereInfo& sphere)
{
    os << "tgSphereInfo(" << sphere.getNode() <<")";
    return os;
}

#endif
