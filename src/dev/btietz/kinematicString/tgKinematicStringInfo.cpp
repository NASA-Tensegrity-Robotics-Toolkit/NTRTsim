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
 * @file tgKinematicStringInfo.cpp
 * @brief Implementation of class tgKinematicStringInfo
 * @author Brian Mirletz
 * @date December 2014
 * $Id$
 */

#include "tgKinematicStringInfo.h"

#include "core/Muscle2P.h"
#include "core/muscleAnchor.h"

tgKinematicStringInfo::tgKinematicStringInfo(const tgKinematicString::Config& config) : 
m_config(config),
tgLinearStringInfo(config)
{}

tgKinematicStringInfo::tgKinematicStringInfo(const tgKinematicString::Config& config, tgTags tags) : 
m_config(config),
tgLinearStringInfo(config, tags)
{}

tgKinematicStringInfo::tgKinematicStringInfo(const tgKinematicString::Config& config, const tgPair& pair) :
m_config(config),
tgLinearStringInfo(config, pair)
{}
    

tgConnectorInfo* tgKinematicStringInfo::createConnectorInfo(const tgPair& pair)
{
    return new tgKinematicStringInfo(m_config, pair);
}

tgModel* tgKinematicStringInfo::createModel(tgWorld& world)
{
    // Don't have to do anything in the world for a Muscle2P...
    // @todo: set the name based on joined tags, or remove name from the model...
    //std::cout << "tgKinematicStringInfo::createModel" << std::endl;
    
    // ensure connector has been initialized
    assert(m_muscle2P);
    return new tgKinematicString(m_muscle2P, getTags(), m_config);
}

