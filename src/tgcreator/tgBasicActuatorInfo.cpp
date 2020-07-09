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
 * @file tgBasicActuatorInfo.cpp
 * @brief Implementation of class tgBasicActuatorInfo
 * @author Ryan Adams
 * @date March 2014
 * $Id$
 */

#include "tgBasicActuatorInfo.h"

#include "core/tgBulletSpringCable.h"
#include "core/tgBulletSpringCableAnchor.h"

tgBasicActuatorInfo::tgBasicActuatorInfo(const tgBasicActuator::Config& config) : 
m_config(config),
tgConnectorInfo()
{}

tgBasicActuatorInfo::tgBasicActuatorInfo(const tgBasicActuator::Config& config, tgTags tags) : 
m_config(config),
tgConnectorInfo(tags)
{}

tgBasicActuatorInfo::tgBasicActuatorInfo(const tgBasicActuator::Config& config, const tgPair& pair) :
m_config(config),
tgConnectorInfo(pair)
{}
    

tgConnectorInfo* tgBasicActuatorInfo::createConnectorInfo(const tgPair& pair)
{
    return new tgBasicActuatorInfo(m_config, pair);
}

void tgBasicActuatorInfo::initConnector(tgWorld& world)
{
    // Note: tgBulletSpringCable holds pointers to things in the world, but it doesn't actually have any in-world representation.
    m_bulletSpringCable = createTgBulletSpringCable();
}

tgModel* tgBasicActuatorInfo::createModel(tgWorld& world)
{
    // Don't have to do anything in the world for a tgBulletSpringCable...
    // @todo: set the name based on joined tags, or remove name from the model...
    //std::cout << "tgBasicActuatorInfo::createModel" << std::endl;
    
    // ensure connector has been initialized
    assert(m_bulletSpringCable);
    return new tgBasicActuator(m_bulletSpringCable, getTags(), m_config);
}

double tgBasicActuatorInfo::getMass() 
{
    // @todo: calculate a mass? tgBulletSpringCable doesn't have physics...
    return 0;
}


tgBulletSpringCable* tgBasicActuatorInfo::createTgBulletSpringCable()
{
     
    // @todo: need to check somewhere that the rigid bodies have been set...
    btRigidBody* fromBody = getFromRigidBody();
    btRigidBody* toBody = getToRigidBody();

    // This method can create the spring-cable either at the node location
    // as specified, or it can automatically re-locate either anchor end
    // to the edge of a rigid body.
    btVector3 from;
    btVector3 to;

    // Choose either the node location (as given by the tgConnectorInfo's point),
    // or the point returned by the attached rigid body's getConnectorInfo method.
    
    // Point "A" is the "From" point, the first btVector3 in the pair.
    if( m_config.moveCablePointAToEdge ){
      from = getFromRigidInfo()->getConnectionPoint(getFrom(), getTo(),
						    m_config.rotation);
    }
    else {
      // The getFrom method is inherited from tgConnectorInfo.
      from = getFrom();
    }
    // Point "B" is the "To" point, the second btVector3 in the pair.
    if( m_config.moveCablePointBToEdge ){
      to = getToRigidInfo()->getConnectionPoint(getTo(), getFrom(),
						m_config.rotation);
    }
    else {
      // The getTo method is inherited from tgConnectorInfo.
      to = getTo();
    }    

    // Older version of this code: always relocate the anchors.
    //btVector3 from = getFromRigidInfo()->getConnectionPoint(getFrom(), getTo(), m_config.rotation);
    //btVector3 to = getToRigidInfo()->getConnectionPoint(getTo(), getFrom(), m_config.rotation);
	
    std::vector<tgBulletSpringCableAnchor*> anchorList;
	
    tgBulletSpringCableAnchor* anchor1 = new tgBulletSpringCableAnchor(fromBody, from);
    anchorList.push_back(anchor1);
	
    tgBulletSpringCableAnchor* anchor2 = new tgBulletSpringCableAnchor(toBody, to);
    anchorList.push_back(anchor2);
	
    return new tgBulletSpringCable(anchorList, m_config.stiffness, m_config.damping, m_config.pretension);
}
    
