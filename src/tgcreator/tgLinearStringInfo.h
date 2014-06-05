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
 * @file tgLinearStringInfo.h
 * @brief Definition of class tgLinearStringInfo
 * @date March 2014
 * $Id$
 */

#ifndef TG_LINEAR_STRING_INFO_H
#define TG_LINEAR_STRING_INFO_H

#include "tgConnectorInfo.h"

#include "tgRigidInfo.h"

#include <string>

#include "core/tgLinearString.h"
#include "core/tgTags.h"

class Muscle2P;

class tgLinearStringInfo : public tgConnectorInfo
{
public:

    /**
     * Construct a tgLinearStringInfo with just a config. The pair must be filled in 
     * later, or factory methods can be used to create instances with
     * pairs.
     */
    tgLinearStringInfo(const tgLinearString::Config& config);

    /**
     * Construct a tgLinearStringInfo with just a config and tags. The pair must 
     * be filled in later, or factory methods can be used to create instances 
     * with pairs.
     */
    tgLinearStringInfo(const tgLinearString::Config& config, tgTags tags);

    /**
     * Construct a tgLinearStringInfo from its endpoints, radius and density.
     * @param[in] from one endpoint
     * @param[in] to the other endpoint
     * @param[in] config contains the radius and density
     * @todo: make sure that tgPairs returns references to the vectors...
     */
    tgLinearStringInfo(const tgLinearString::Config& config, const tgPair& pair);
    

    virtual ~tgLinearStringInfo() {}
    
    /**
     * Create a tgConnectorInfo* from a tgPair
     */ 
    virtual tgConnectorInfo* createConnectorInfo(const tgPair& pair);

    void initConnector(tgWorld& world);

    virtual tgModel* createModel(tgWorld& world);

    double getMass();

private:    
    
    Muscle2P* createMuscle2P();
    
private:
    
    tgLinearString::Config m_config;
    Muscle2P* m_muscle2P;
};


#endif
