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

#ifndef TG_RB_STRING_INFO_H
#define TG_RB_STRING_INFO_H

/**
 * @file tgRBStringInfo.h
 * @brief Contains the definition of class tgRBStringInfo. A string with
 * small rigid bodies to create contact dynamics. Depricated as of
 * version 1.1.0
 * @author Brian Tietz
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

#include "tgcreator/tgConnectorInfo.h" 
// This library
#include "tgRBString.h"

class tgWorld;
class CPGEquations;
class tgNode;

class tgRBStringInfo: public tgConnectorInfo
{
public: 
       
    tgRBStringInfo(const tgRBString::Config& config);
    
    tgRBStringInfo(const tgRBString::Config& config, tgTags tags);

    tgRBStringInfo(const tgRBString::Config& config, const tgPair& pair);
    
    virtual ~tgRBStringInfo()
    {}
    
    /**
     * Create a tgConnectorInfo* from a tgPair
     */ 
    virtual tgConnectorInfo* createConnectorInfo(const tgPair& pair)
    {
        return new tgRBStringInfo(m_config, pair);
    }
    
    // Anything to do here?
    void initConnector(tgWorld& world) {}
    
    virtual tgModel* createModel(tgWorld& world);
    
    const int getSegments() const
    {
        return m_config.m_segments;
    }
    
    // This isn't called by anything. Will we really need it for connectors?
    double getMass() 
    {
        // @todo: add up the rigid bodies
        return 0;
    }

protected:

    void buildModel(tgWorld& world, tgModel* thisString); 

    tgRBString::Config m_config;
    
};

#endif // TG_RB_STRING_INFO_H
