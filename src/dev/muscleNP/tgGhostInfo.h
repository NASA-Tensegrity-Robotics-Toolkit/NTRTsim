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
#include "tgcreator/tgBoxInfo.h"
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
 * Builds off of tgRigidInfo and tgBoxInfo, but creates ghost objects instead of
 * rigid bodies. Needs to be in its own seperate structure, so
 * these don't get accidentally compounded with rigid objects.
 * Used by MuscleNP and tgMultiPointStringInfo
 */ 
class tgGhostInfo : public tgBoxInfo {
public:
        
    /**
     * Construct a tgBoxInfo with just a config. The pair must be filled in 
     * later, or factory methods can be used to create instances with
     * pairs.
     */
    tgGhostInfo(const tgBox::Config& config);

    /**
     * Construct a tgBoxInfo with just a config and tags. The pair must 
     * be filled in later, or factory methods can be used to create instances 
     * with pairs.
     */
    tgGhostInfo(const tgBox::Config& config, tgTags tags);

    /**
     * Construct a tgBoxInfo from its endpoints, radius and density.
     * @param[in] from one endpoint
     * @param[in] to the other endpoint
     * @param[in] config contains the radius and density
     * @todo: make sure that tgPairs returns references to the vectors...
     */
    tgGhostInfo(const tgBox::Config& config, const tgPair& pair);

    /**
     * Construct a tgBoxInfo from its endpoints, radius and density.
     * @param[in] from one endpoint
     * @param[in] to the other endpoint
     * @param[in] config contains the radius and density
     * @todo: make sure that tgPairs returns references to the vectors...
     */
    tgGhostInfo(const tgBox::Config& config, tgTags tags, const tgPair& pair);
    
    /**
     * World will destroy the rigid body
     */
    virtual ~tgGhostInfo() {} 
    
    /**
     * Create a tgRigidInfo* from a tgPair
     */ 
    tgRigidInfo* createRigidInfo(const tgPair& pair);
	
    virtual void initRigidBody(tgWorld& world);

    virtual tgModel* createModel(tgWorld& world);

#if (0) // Default to box's com so we can do compounding
	/**
     * Return the ghost object's mass
     * Ghost objects have no mass
     * @return the mass of the ghost object
     */
    virtual double getMass() const
    {
		return 0;
	}
#endif
};

/**
 * Overload operator<<() to handle a tgBoxInfo.
 * @param[in,out] os an ostream
 * @param[in] q a tgBoxInfo
 * @return os
 */
inline std::ostream& operator<<(std::ostream& os, const tgGhostInfo& casper)
{
    os << "tgGhostInfo(" << casper.getFrom() << ", " << casper.getTo() <<")";
    return os;
}


#endif
