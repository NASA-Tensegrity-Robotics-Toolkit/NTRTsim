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
 * @file tgUnidirComprSprActuatorInfo.cpp
 * @brief Implementation of class tgUnidirComprSprActuatorInfo
 * @author Drew Sabelhaus, et al.
 * @date August 2016
 * $Id$
 */

// This class
#include "tgUnidirComprSprActuatorInfo.h"

// Other classes from core (are these included from the superclass?...)
//#include "core/tgBulletCompressionSpring.h"
#include "core/tgBulletSpringCableAnchor.h"
#include "core/tgCast.h"

// Include the new type of spring
#include "core/tgBulletUnidirComprSpr.h"

/**
 * Constructor auxiliary method, does some debugging and error checking.
 */
void tgUnidirComprSprActuatorInfo::constructorAux()
{
    // Debugging
    #if (0)
    std::cout << "tgUnidirComprSprActuatorInfo constructor. Direction is:" << std::endl;
    std::cout << "(" << m_config.direction->x() << ",";
    std::cout << m_config.direction->y() << ",";
    std::cout << m_config.direction->z() << ")" << std::endl;
    #endif 
}

/**
 * Constructors.
 * All must call the constructor for tgCompressionSpringActuatorInfo.
 */

tgUnidirComprSprActuatorInfo::tgUnidirComprSprActuatorInfo(const tgUnidirComprSprActuator::Config& config) : 
m_config(config),
tgCompressionSpringActuatorInfo(config)
{
  // call the helper function for the constructors
  constructorAux();
}

tgUnidirComprSprActuatorInfo::tgUnidirComprSprActuatorInfo(const tgUnidirComprSprActuator::Config& config, tgTags tags) : 
m_config(config),
tgCompressionSpringActuatorInfo(config, tags)
{
  // call the helper function for the constructors
  constructorAux();
}

tgUnidirComprSprActuatorInfo::tgUnidirComprSprActuatorInfo(const tgUnidirComprSprActuator::Config& config, const tgPair& pair) :
m_config(config),
tgCompressionSpringActuatorInfo(config, pair)
{
  // call the helper function for the constructors
  constructorAux();
}

tgConnectorInfo* tgUnidirComprSprActuatorInfo::createConnectorInfo(const tgPair& pair)
{
    return new tgUnidirComprSprActuatorInfo(m_config, pair);
}

void tgUnidirComprSprActuatorInfo::initConnector(tgWorld& world)
{
    // Note: tgBulletUnidirComprSpr holds pointers to things
    // in the world, but it doesn't actually have any in-world representation.
    // Remember that m_bulletCompressionSpring is held in the superclass.
    m_bulletCompressionSpring = createTgBulletUnidirComprSpr();
}

tgModel* tgUnidirComprSprActuatorInfo::createModel(tgWorld& world)
{
    // Don't have to do anything in the world for a tgBulletCompressionSpring...
    // @todo: set the name based on joined tags, or remove name from the model...
    //std::cout << "tgUnidirComprSprActuatorInfo::createModel" << std::endl;
    
    // ensure connector has been initialized
    assert(m_bulletCompressionSpring);

    // Here, we have to cast the compression spring to the type that's used in this
    // class. Casting from the superclass (tgBulletCompressionSpring) to the
    // subclass tgBulletUnidirComprSpr.
    // Rememeber that m_bulletCompressionSpring is a pointer to
    // a bulletCompressionSpring.
    return new tgUnidirComprSprActuator(
	 tgCast::cast<tgBulletCompressionSpring, tgBulletUnidirComprSpr>(m_bulletCompressionSpring), getTags(), m_config);
}

tgBulletUnidirComprSpr* tgUnidirComprSprActuatorInfo::createTgBulletUnidirComprSpr()
{
    // @TO-DO: make this more object-oriented. Currently, there is code re-use
    // between this method and the create compression spring method in
    // the parent class.
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
      from = getFromRigidInfo()->getConnectionPoint(getFrom(), getTo());
    }
    else {
      // The getFrom method is inherited from tgConnectorInfo.
      from = getFrom();
    }
    // Point "B" is the "To" point, the second btVector3 in the pair.
    if( m_config.moveCablePointBToEdge ){
      to = getToRigidInfo()->getConnectionPoint(getTo(), getFrom());
    }
    else {
      // The getTo method is inherited from tgConnectorInfo.
      to = getTo();
    }    

    // Note that even though this object is definitely not a SpringCable,
    // and does not inherit from that class at all, it's still more useful
    // to re-use the spring cable version of anchor.
    // No need to re-create the exact same object, just to name it differently.
    // This should work, since tgBulletSpringCableAnchor actually has
    // no tie-ins to either tgBulletSpringCable or tgSpringCable.
    std::vector<tgBulletSpringCableAnchor*> anchorList;

    // add the anchors from these two rigid bodies to the list of anchors that
    // will reside within the tgBulletCompressionSpring.
    tgBulletSpringCableAnchor* anchor1 =
      new tgBulletSpringCableAnchor(fromBody, from);
    anchorList.push_back(anchor1);
	
    tgBulletSpringCableAnchor* anchor2 =
      new tgBulletSpringCableAnchor(toBody, to);
    anchorList.push_back(anchor2);

    // call the constructor for tgBulletUnidirComprSpr.
    // Unlike the spring-cable, it makes more sense to state the rest length
    // of a compression spring. That way, it's easy to only apply a force
    // if the total distance between the two anchors is less than restLength.
    return new tgBulletUnidirComprSpr(anchorList,
		  m_config.isFreeEndAttached, m_config.stiffness, m_config.damping,
		  m_config.restLength, m_config.direction);
}

