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

#ifndef TG_TOUCH_SENSOR_SPHERE_INFO_H
#define TG_TOUCH_SENSOR_SPHERE_INFO_H

/**
 * @file tgTouchSensorSphereInfo.h
 * @brief Definition of abstract class tgTouchSensorSphereInfo
 * @author Alexander Xydes
 * @date November 2014
 * $Id$
 */

// The C++ Standard Library
#include <set>
// This library
#include "core/tgTaggable.h"
#include "core/tgModel.h"
#include "tgcreator/tgRigidInfo.h"
#include "tgcreator/tgSphereInfo.h"
//Bullet Physics
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"

// Forward references
class tgCompoundRigidInfo;
class tgNode;
class tgNodes;
class tgTagSearch;
class tgWorld;

class btRigidBody;
class btCollisionShape;
class btTransform;

/**
 * Builds off of tgRigidInfo and tgSphereInfo, but creates ghost objects instead of
 * rigid bodies. Needs to be in its own seperate structure, so
 * these don't get accidentally compounded with rigid objects.
 * Used by MuscleNP and tgMultiPointStringInfo
 */ 
class tgTouchSensorSphereInfo : public tgSphereInfo {
public:
        
    /**
     * Construct a tgSphereInfo with just a config. The pair must be filled in
     * later, or factory methods can be used to create instances with
     * pairs.
     */
    tgTouchSensorSphereInfo(const tgSphere::Config& config);

    /**
     * Construct a tgSphereInfo with just a config and tags. The pair must
     * be filled in later, or factory methods can be used to create instances 
     * with pairs.
     */
    tgTouchSensorSphereInfo(const tgSphere::Config& config, tgTags tags);

    /**
     * Construct a tgSphereInfo from its endpoints, radius and density.
     * @param[in] the center point
     * @param[in] config contains the radius and density
     * @todo: make sure that tgPairs returns references to the vectors...
     */
    tgTouchSensorSphereInfo(const tgSphere::Config& config, const tgNode &node);

    /**
     * Construct a tgSphereInfo from its endpoints, radius and density.
     * @param[in] the center point
     * @param[in] config contains the radius and density
     * @todo: make sure that tgPairs returns references to the vectors...
     */
    tgTouchSensorSphereInfo(const tgSphere::Config& config, tgTags tags, const tgNode &node);
    
    /**
     * World will destroy the rigid body
     */
    virtual ~tgTouchSensorSphereInfo() {}
    
    /**
     * Create a tgRigidInfo* from a tgNode
     */ 
    tgRigidInfo* createRigidInfo(const tgNode& node);
	
    virtual void initRigidBody(tgWorld& world);

    virtual tgModel* createModel(tgWorld& world);
};

/**
 * Overload operator<<() to handle a tgSphereInfo.
 * @param[in,out] os an ostream
 * @param[in] q a tgSphereInfo
 * @return os
 */
inline std::ostream& operator<<(std::ostream& os, const tgTouchSensorSphereInfo& casper)
{
    os << "tgTouchSensorSphereInfo(" << casper.getNode() <<")";
    return os;
}


#endif
