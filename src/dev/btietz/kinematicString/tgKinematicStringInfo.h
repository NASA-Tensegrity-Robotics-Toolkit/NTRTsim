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

/**
 * @file tgKinematicStringInfo.h
 * @brief Definition of class tgKinematicStringInfo
 * @author Brian Mirletz
 * @date December 2014
 * $Id$
 */

#ifndef TG_KINEMATIC_STRING_INFO_H
#define TG_KINEMATIC_STRING_INFO_H

#include "tgcreator/tgLinearStringInfo.h"

#include "tgcreator/tgConnectorInfo.h"
#include "tgcreator/tgRigidInfo.h"

#include <string>

#include "tgKinematicString.h"
#include "core/tgTags.h"

class tgBulletSpringCable;

class tgKinematicStringInfo : public tgLinearStringInfo
{
public:

    /**
     * Construct a tgKinematicStringInfo with just a config. The pair must be filled in 
     * later, or factory methods can be used to create instances with
     * pairs.
     */
    tgKinematicStringInfo(const tgKinematicString::Config& config);

    /**
     * Construct a tgKinematicStringInfo with just a config and tags. The pair must 
     * be filled in later, or factory methods can be used to create instances 
     * with pairs.
     */
    tgKinematicStringInfo(const tgKinematicString::Config& config, tgTags tags);

    /**
     * Construct a tgKinematicStringInfo from its endpoints, radius and density.
     * @param[in] from one endpoint
     * @param[in] to the other endpoint
     * @param[in] config contains the radius and density
     * @todo: make sure that tgPairs returns references to the vectors...
     */
    tgKinematicStringInfo(const tgKinematicString::Config& config, const tgPair& pair);
    

    virtual ~tgKinematicStringInfo() {}
    
    /**
     * Create a tgConnectorInfo* from a tgPair
     */ 
    virtual tgConnectorInfo* createConnectorInfo(const tgPair& pair);

    virtual tgModel* createModel(tgWorld& world);


private:
    
    tgKinematicString::Config m_config;

};


#endif // TG_KINEMATIC_STRING_INFO_H
