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
 * @file tgBuildSpec.h
 * @brief Definition of class tgBuildSpec
 * @author Ryan Adams
 * @date March 2014
 * $Id$
 */

#ifndef TG_BUILD_SPEC_H
#define TG_BUILD_SPEC_H

#include <vector>

#include "core/tgTagSearch.h"

class tgRigidInfo;
class tgConnectorInfo;

/**
 * Contains a search and a builder, to be applied to something like a structure (passed in to the contained builder).
 * The contained builder can create a tgRigidInfo or a tgConnectorInfo, which will be placed where needed by the caller.
 */
class tgBuildSpec
{
public:
    struct RigidAgent
    {
    public:
        RigidAgent(tgTagSearch s, tgRigidInfo* b) : tagSearch(s), infoFactory(b) 
        {}
        RigidAgent(std::string s, tgRigidInfo* b) : tagSearch(tgTagSearch(s)), infoFactory(b) 
        {}
        ~RigidAgent();
        
        tgTagSearch tagSearch;

        // We own this (@todo: do we?)
        tgRigidInfo* infoFactory;
    };

    struct ConnectorAgent
    {
    public:
        ConnectorAgent(tgTagSearch s, tgConnectorInfo* b) : tagSearch(s), infoFactory(b) 
        {}
        ConnectorAgent(std::string s, tgConnectorInfo* b) : tagSearch(tgTagSearch(s)), infoFactory(b) 
        {}
        ~ConnectorAgent();

        tgTagSearch tagSearch;

        // We own this (@todo: do we?)
        tgConnectorInfo* infoFactory;
    };

    tgBuildSpec() {}
    virtual ~tgBuildSpec();

    void addBuilder(std::string tag_search, tgRigidInfo* infoFactory);
    
    void addBuilder(std::string tag_search, tgConnectorInfo* infoFactory);
    
    std::vector<RigidAgent*> getRigidAgents()
    {
        return m_rigidAgents;
    }
    
    std::vector<ConnectorAgent*> getConnectorAgents()
    {
        return m_connectorAgents;
    }
    
private:
    std::vector<RigidAgent*> m_rigidAgents;
    std::vector<ConnectorAgent*> m_connectorAgents;  
};

#endif
