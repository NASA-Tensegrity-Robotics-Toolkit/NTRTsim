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
 * @file tgKinematicContactCableInfo.cpp
 * @brief Implementation of class tgKinematicContactCableInfo
 * @author Brian Mirletz
 * @date December 2014
 * $Id$
 */

#include "tgKinematicContactCableInfo.h"

#include "core/tgBulletContactSpringCable.h"

tgKinematicContactCableInfo::tgKinematicContactCableInfo(const tgKinematicActuator::Config& config) : 
m_config(config),
tgBasicContactCableInfo(config)
{}

tgKinematicContactCableInfo::tgKinematicContactCableInfo(const tgKinematicActuator::Config& config, tgTags tags) : 
m_config(config),
tgBasicContactCableInfo(config, tags)
{}

tgKinematicContactCableInfo::tgKinematicContactCableInfo(const tgKinematicActuator::Config& config, const tgPair& pair) :
m_config(config),
tgBasicContactCableInfo(config, pair)
{}
    

tgConnectorInfo* tgKinematicContactCableInfo::createConnectorInfo(const tgPair& pair)
{
    return new tgKinematicContactCableInfo(m_config, pair);
}

tgModel* tgKinematicContactCableInfo::createModel(tgWorld& world)
{
    // Don't have to do anything in the world for a Muscle2P...
    // @todo: set the name based on joined tags, or remove name from the model...
    //std::cout << "tgKinematicContactCableInfo::createModel" << std::endl;
    
    // ensure connector has been initialized
    assert(m_bulletContactSpringCable);
    return new tgKinematicActuator(m_bulletContactSpringCable, getTags(), m_config);
}

