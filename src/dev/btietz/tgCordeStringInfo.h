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
 * @file tgCordeStringInfo.h
 * @brief Definition of class tgLinearStringInfo
 * @author Ryan Adams
 * @date March 2014
 * $Id$
 */

#ifndef TG_CORDE_STRING_INFO_H
#define TG_CORDE_STRING_INFO_H

#include "tgcreator/tgConnectorInfo.h"

#include "tgcreator/tgRigidInfo.h"

#include <string>

#include "tgCordeModel.h"
#include "core/tgTags.h"

#include "dev/Corde/CordeModel.h"

class cordeCollisionObject;
class tgWorld;

class tgCordeStringInfo : public tgConnectorInfo
{
public:

    /**
     * Construct a tgCordeStringInfo with just a config. The pair must be filled in 
     * later, or factory methods can be used to create instances with
     * pairs.
     */
    tgCordeStringInfo(const CordeModel::Config& config);

    /**
     * Construct a tgCordeStringInfo with just a config and tags. The pair must 
     * be filled in later, or factory methods can be used to create instances 
     * with pairs.
     */
    tgCordeStringInfo(const CordeModel::Config& config, tgTags tags);

    /**
     * Construct a tgCordeStringInfo from its endpoints, radius and density.
     * @param[in] from one endpoint
     * @param[in] to the other endpoint
     * @param[in] config contains the radius and density
     * @todo: make sure that tgPairs returns references to the vectors...
     */
    tgCordeStringInfo(const CordeModel::Config& config, const tgPair& pair);
    

    virtual ~tgCordeStringInfo() {}
    
    /**
     * Create a tgConnectorInfo* from a tgPair
     */ 
    virtual tgConnectorInfo* createConnectorInfo(const tgPair& pair);

    void initConnector(tgWorld& world);

    virtual tgModel* createModel(tgWorld& world);

    double getMass();

private:    
    
    cordeCollisionObject* createCordeString(tgWorld& world);
    
    std::vector<btVector3> generatePoints(btVector3& point1, btVector3& point2, std::size_t resolution);
    
private:
    
    CordeModel::Config m_config;
    cordeCollisionObject* m_cordeString;
};


#endif
