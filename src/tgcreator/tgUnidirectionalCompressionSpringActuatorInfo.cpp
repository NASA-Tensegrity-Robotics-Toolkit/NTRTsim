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
 * @file tgUnidirectionalCompressionSpringActuatorInfo.cpp
 * @brief Implementation of class tgUnidirectionalCompressionSpringActuatorInfo
 * @author Drew Sabelhaus, Ryan Adams
 * @date August 2016
 * $Id$
 */

// This class
#include "tgUnidirectionalCompressionSpringActuatorInfo.h"

// Other classes from core (are these included from the superclass?...)
//#include "core/tgBulletCompressionSpring.h"
#include "core/tgBulletSpringCableAnchor.h"
#include "core/tgCast.h"

// Include the new type of spring
#include "core/tgBulletUnidirectionalCompressionSpring.h"

/**
 * Constructor auxiliary method, does some debugging and error checking.
 */
void tgUnidirectionalCompressionSpringActuatorInfo::constructorAux()
{
    // Debugging
    #if (0)
    std::cout << "tgUnidirectionalCompressionSpringActuatorInfo constructor. Direction is:" << std::endl;
    std::cout << "(" << m_config.direction->x() << ",";
    std::cout << m_config.direction->y() << ",";
    std::cout << m_config.direction->z() << ")" << std::endl;
    #endif 
}

/**
 * Constructors.
 * All must call the constructor for tgCompressionSpringActuator.
 */

tgUnidirectionalCompressionSpringActuatorInfo::tgUnidirectionalCompressionSpringActuatorInfo(const tgUnidirectionalCompressionSpringActuator::Config& config) : 
m_config(config),
tgCompressionSpringActuatorInfo(config)
{
  // call the helper function for the constructors
  constructorAux();
}

tgUnidirectionalCompressionSpringActuatorInfo::tgUnidirectionalCompressionSpringActuatorInfo(const tgUnidirectionalCompressionSpringActuator::Config& config, tgTags tags) : 
m_config(config),
tgCompressionSpringActuatorInfo(config, tags)
{
  // call the helper function for the constructors
  constructorAux();
}

tgUnidirectionalCompressionSpringActuatorInfo::tgUnidirectionalCompressionSpringActuatorInfo(const tgUnidirectionalCompressionSpringActuator::Config& config, const tgPair& pair) :
m_config(config),
tgCompressionSpringActuatorInfo(config, pair)
{
  // call the helper function for the constructors
  constructorAux();
}

tgConnectorInfo* tgUnidirectionalCompressionSpringActuatorInfo::createConnectorInfo(const tgPair& pair)
{
    return new tgUnidirectionalCompressionSpringActuatorInfo(m_config, pair);
}

void tgUnidirectionalCompressionSpringActuatorInfo::initConnector(tgWorld& world)
{
    // Note: tgBulletUnidirectionalCompressionSpring holds pointers to things
    // in the world, but it doesn't actually have any in-world representation.
    // Remember that m_bulletCompressionSpring is held in the superclass.
    m_bulletCompressionSpring = createTgBulletUnidirectionalCompressionSpring();
}

tgModel* tgUnidirectionalCompressionSpringActuatorInfo::createModel(tgWorld& world)
{
    // Don't have to do anything in the world for a tgBulletCompressionSpring...
    // @todo: set the name based on joined tags, or remove name from the model...
    //std::cout << "tgUnidirectionalCompressionSpringActuatorInfo::createModel" << std::endl;
    
    // ensure connector has been initialized
    assert(m_bulletCompressionSpring);

    // Here, we have to cast the compression spring to the type that's used in this
    // class. Casting from the superclass (tgBulletCompressionSpring) to the
    // subclass tgBulletUnidirectionalCompressionSpring.
    // Rememeber that m_bulletCompressionSpring is a pointer to
    // a bulletCompressionSpring.
    return new tgUnidirectionalCompressionSpringActuator(
	 tgCast::cast<tgBulletCompressionSpring, tgBulletUnidirectionalCompressionSpring>(m_bulletCompressionSpring), getTags(), m_config);
}

tgBulletUnidirectionalCompressionSpring* tgUnidirectionalCompressionSpringActuatorInfo::createTgBulletUnidirectionalCompressionSpring()
{
  //std::cout << "tgUnidirectionalCompressionSpringActuatorInfo::createTgBulletUnidirectionalCompressionSpring()" << std::endl;
    
    //std::cout << "  getFromRigidInfo(): " << getFromRigidInfo() << std::endl;
    //std::cout << "  getFromRigidInfo(): " << getFromRigidInfo()->getRigidInfoGroup() << std::endl;
    
    // @todo: need to check somewhere that the rigid bodies have been set...
    btRigidBody* fromBody = getFromRigidBody();
    btRigidBody* toBody = getToRigidBody();

    // get the two rigid bodies that correspond to the locations that this compression spring
    // will be attached between.
    btVector3 from = getFromRigidInfo()->getConnectionPoint(getFrom(), getTo());
    btVector3 to = getToRigidInfo()->getConnectionPoint(getTo(), getFrom());

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

    // call the constructor for tgBulletUnidirectionalCompressionSpring.
    // Unlike the spring-cable, it makes more sense to state the rest length of a compression
    // spring. That way, it's easy to only apply a force if the total distance between the two
    // anchors is less than restLength.
    return new tgBulletUnidirectionalCompressionSpring(anchorList,
		  m_config.isFreeEndAttached, m_config.stiffness, m_config.damping,
		  m_config.restLength, m_config.direction);
}

