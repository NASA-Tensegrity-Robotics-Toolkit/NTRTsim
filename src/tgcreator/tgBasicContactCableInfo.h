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
 * @file tgBasicContactCableInfo.h
 * @brief Definition of class tgBasicContactCableInfo
 * @author Brian Mirletz and Ryan Adams
 * @date October 2014
 * $Id$
 */

#ifndef SRC_TGCREATOR_TG_BASIC_CONTACT_CABLE_INFO_H
#define SRC_TGCREATOR_TG_BASIC_CONTACT_CABLE_INFO_H

#include "tgcreator/tgConnectorInfo.h"

#include "tgcreator/tgRigidInfo.h"

#include <string>

#include "core/tgBasicActuator.h"
#include "core/tgTags.h"

class tgBulletContactSpringCable;

class tgBasicContactCableInfo : public tgConnectorInfo
{
public:

    /**
     * Construct a tgBasicContactCableInfo with just a config. The pair must be filled in 
     * later, or factory methods can be used to create instances with
     * pairs.
     */
    tgBasicContactCableInfo(const tgBasicActuator::Config& config);

    /**
     * Construct a tgBasicContactCableInfo with just a config and tags. The pair must 
     * be filled in later, or factory methods can be used to create instances 
     * with pairs.
     */
    tgBasicContactCableInfo(const tgBasicActuator::Config& config, tgTags tags);

    /**
     * Construct a tgBasicContactCableInfo from its endpoints, radius and density.
     * @param[in] from one endpoint
     * @param[in] to the other endpoint
     * @param[in] config contains the radius and density
     * @todo: make sure that tgPairs returns references to the vectors...
     */
    tgBasicContactCableInfo(const tgBasicActuator::Config& config, const tgPair& pair);
    

    virtual ~tgBasicContactCableInfo() {}
    
    /**
     * Create a tgConnectorInfo* from a tgPair
     */ 
    virtual tgConnectorInfo* createConnectorInfo(const tgPair& pair);

    void initConnector(tgWorld& world);

    virtual tgModel* createModel(tgWorld& world);

    double getMass();

protected:
    tgBulletContactSpringCable* m_bulletContactSpringCable;
    
private:    
    
    tgBulletContactSpringCable* createTgBulletContactSpringCable(tgWorld& world);
    
private:
    
    tgBasicActuator::Config m_config;
    
};


#endif // SRC_TGCREATOR_TG_BASIC_CONTACT_CABLE_INFO_H
