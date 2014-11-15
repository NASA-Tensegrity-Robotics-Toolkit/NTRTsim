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

#ifndef TG_ROD_HINGE_INFO_H
#define TG_ROD_HINGE_INFO_H

/**
 * @file tgRodHingeInfo.h
 * @brief Contains the definition of class tgHingeInfo. A hinge joint builder.
 * @author Alexander Xydes
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

#include "tgRodHinge.h"
#include "tgcreator/tgConnectorInfo.h"

#include "LinearMath/btVector3.h"

class btHingeConstraint;

class tgRodHingeInfo: public tgConnectorInfo
{
public: 
       
    /**
     * Construct a tgHingeInfo with just a config. The pair must be filled in
     * later, or factory methods can be used to create instances with
     * pairs.
     */
    tgRodHingeInfo(const tgRodHinge::Config& config);
    
    /**
     * Construct a tgHingeInfo with just a config and tags. The pair must
     * be filled in later, or factory methods can be used to create instances
     * with pairs.
     */
    tgRodHingeInfo(const tgRodHinge::Config& config, tgTags tags);

    /**
     * Construct a tgHingeInfo from its Config
     * @param[in] from one endpoint
     * @param[in] to the other endpoint
     * @param[in] config
     * @todo: make sure that tgPairs returns references to the vectors...
     */
    tgRodHingeInfo(const tgRodHinge::Config& config, const tgPair& pair);
    
    ~tgRodHingeInfo();
    
    /**
     * Create a tgConnectorInfo* from a tgPair
     */ 
    virtual tgConnectorInfo* createConnectorInfo(const tgPair& pair);
    
    void initConnector(tgWorld& world);
    
    virtual tgModel* createModel(tgWorld& world);
    
    double getMass();

    btHingeConstraint* createHinge();

    btVector3 getRigidVector(bool isCompound, std::set<btVector3> fromNodes);
protected:
    tgRodHinge::Config m_config;
};

#endif
