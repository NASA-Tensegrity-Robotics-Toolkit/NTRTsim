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
 * @file tgBuildSpec.cpp
 * @brief Implementation of class tgBuildSpec
 * @author Ryan Adams
 * @date March 2014
 * $Id$
 */

#include "tgBuildSpec.h"
#include "tgRigidInfo.h"
#include "tgConnectorInfo.h"

#include "core/tgException.h"
#include "core/tgTags.h"
#include "core/tgTagSearch.h"

tgBuildSpec::RigidAgent::~RigidAgent()  
{
    delete infoFactory;
}
tgBuildSpec::ConnectorAgent::~ConnectorAgent()  
{
    delete infoFactory;
}

tgBuildSpec::~tgBuildSpec() {
    while (!m_rigidAgents.empty()){
         RigidAgent* agent = m_rigidAgents.back();
         m_rigidAgents.pop_back();
         delete agent;
     }
     while (!m_connectorAgents.empty()){
          ConnectorAgent* agent = m_connectorAgents.back();
          m_connectorAgents.pop_back();
          delete agent;
      }
}

void tgBuildSpec::addBuilder(std::string tag_search, tgRigidInfo* infoFactory)
{
    // Note: we could try to check that search strings are different here,
    // but we're going to defer that until the build phase due to the 
    // large possibility of unhandled edge cases (e.g. 'a b c' matches some
    // of the same things as 'a b'. Also, 'a|b|c -c' === 'a|b -c', and it
    // goes downhill from there. tgTagSearch should be able to handle that,
    // but user messaging would be difficult here.)
    
    //m_infoFactorys.push_back(tgBuildSpec::Entry(tgTagSearch(tag_search), infoFactory)); // @todo: make this work
    m_rigidAgents.push_back(new RigidAgent(tag_search, infoFactory));
}

void tgBuildSpec::addBuilder(std::string tag_search, tgConnectorInfo* infoFactory)
{
    m_connectorAgents.push_back(new ConnectorAgent(tag_search, infoFactory));
}

