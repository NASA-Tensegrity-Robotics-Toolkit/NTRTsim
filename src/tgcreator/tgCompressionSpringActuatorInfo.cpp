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
 * @file tgCompressionSpringActuatorInfo.cpp
 * @brief Implementation of class tgCompressionSpringActuatorInfo
 * @author Drew Sabelhaus, Ryan Adams
 * @date August 2016
 * $Id$
 */

// This class
#include "tgCompressionSpringActuatorInfo.h"

// Other classes from core
#include "core/tgBulletCompressionSpring.h"
#include "core/tgBulletSpringCableAnchor.h"

tgCompressionSpringActuatorInfo::tgCompressionSpringActuatorInfo(const tgCompressionSpringActuator::Config& config) : 
m_config(config),
tgConnectorInfo()
{}

tgCompressionSpringActuatorInfo::tgCompressionSpringActuatorInfo(const tgCompressionSpringActuator::Config& config, tgTags tags) : 
m_config(config),
tgConnectorInfo(tags)
{}

tgCompressionSpringActuatorInfo::tgCompressionSpringActuatorInfo(const tgCompressionSpringActuator::Config& config, const tgPair& pair) :
m_config(config),
tgConnectorInfo(pair)
{}
    

tgConnectorInfo* tgCompressionSpringActuatorInfo::createConnectorInfo(const tgPair& pair)
{
    return new tgCompressionSpringActuatorInfo(m_config, pair);
}

void tgCompressionSpringActuatorInfo::initConnector(tgWorld& world)
{
    // Note: tgBulletCompressionSpring holds pointers to things in the world, but it doesn't actually have any in-world representation.
    m_bulletCompressionSpring = createTgBulletCompressionSpring();
}

tgModel* tgCompressionSpringActuatorInfo::createModel(tgWorld& world)
{
    // Don't have to do anything in the world for a tgBulletCompressionSpring...
    // @todo: set the name based on joined tags, or remove name from the model...
    //std::cout << "tgCompressionSpringActuatorInfo::createModel" << std::endl;
    
    // ensure connector has been initialized
    assert(m_bulletCompressionSpring);
    return new tgCompressionSpringActuator(m_bulletCompressionSpring, getTags(), m_config);
}

double tgCompressionSpringActuatorInfo::getMass() 
{
    // @todo: this should never be called, unless we wanted to model the mass of
    // a compression spring at some point. For now, maybe throw an error?
    return 0;
}


tgBulletCompressionSpring* tgCompressionSpringActuatorInfo::createTgBulletCompressionSpring()
{
    std::cout << "tgCompressionSpringActuatorInfo::createTgBulletCompressionSpring()" << std::endl;
    
    std::cout << "  getFromRigidInfo(): " << getFromRigidInfo() << std::endl;
    std::cout << "  getFromRigidInfo(): " << getFromRigidInfo()->getRigidInfoGroup() << std::endl;
    
    // @todo: need to check somewhere that the rigid bodies have been set...
    btRigidBody* fromBody = getFromRigidBody();
    btRigidBody* toBody = getToRigidBody();

    // get the two rigid bodies that correspond to the locations that this compression spring
    // will be attached between.
    btVector3 from = getFromRigidInfo()->getConnectionPoint(getFrom(), getTo(), m_config.rotation);
    btVector3 to = getToRigidInfo()->getConnectionPoint(getTo(), getFrom(), m_config.rotation);

    // Note that even though this object is definitely not a SpringCable, and does not inherit from
    // that class at all, it's still more useful to re-use the spring cable version of anchor.
    // no need to re-create the exact same object, just to name it differently.
    // This should work, since tgBulletSpringCableAnchor actually has no tie-ins to either
    // tgBulletSpringCable or tgSpringCable.
    std::vector<tgBulletSpringCableAnchor*> anchorList;

    // add the anchors from these two rigid bodies to the list of anchors that
    // will reside within the tgBulletCompressionSpring.
    tgBulletSpringCableAnchor* anchor1 = new tgBulletSpringCableAnchor(fromBody, from);
    anchorList.push_back(anchor1);
	
    tgBulletSpringCableAnchor* anchor2 = new tgBulletSpringCableAnchor(toBody, to);
    anchorList.push_back(anchor2);

    // call the constructor fot tgBulletCompressionSpring.
    // Unlike the spring-cable, it makes more sense to state the rest length of a compression
    // spring. That way, it's easy to only apply a force if the total distance between the two
    // anchors is less than restLength.
    return new tgBulletCompressionSpring(anchorList, m_config.stiffness, m_config.damping, m_config.restLength);
}
    
