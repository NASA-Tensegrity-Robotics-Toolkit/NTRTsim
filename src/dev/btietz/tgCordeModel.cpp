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

#include "tgCordeModel.h"

#include "core/tgModelVisitor.h"
#include "core/tgWorld.h"

#include "dev/Corde/CordeModel.h"
#include "dev/Corde/cordeCollisionObject.h"

tgCordeModel::Config::Config(tgBaseString::Config motor_config,
								CordeModel::Config string_config) :
motorConfig(motor_config),
stringConfig(string_config)
{
	// Assertions should be handled by the respective configs.
}

tgCordeModel::tgCordeModel(cordeCollisionObject* string,
							tgBaseString::Config motor_config,
							const tgTags& tags) :
m_string(string),
tgBaseString(tags, motor_config, string->getRestLength(), string->getActualLength())
{	

}
    
tgCordeModel::~tgCordeModel()
{
    
}
    
void tgCordeModel::setup(tgWorld& world)
{
    notifySetup();
    tgModel::setup(world);
}

void tgCordeModel::teardown()
{
    // World handles deleting collision object
    m_string = NULL;
    tgModel::teardown();
}

#if (0)    
void tgCordeModel::step(double dt)
{
	//testString->applyForce(btVector3(0, 0.0, -90.0), 0);
	//testString->applyForce(btVector3(0.0, 0.0, 90.0), 9);
	testString->applyUniformAcc(btVector3(0.0, -9.81, 0.0));
	//testString->applyVecTorque(btVector3(100.0, 100.0, 0.0), 0);
	//testString->applyVecTorque(btVector3(0.0, 0.0, -100.0), 28);
    //testString->step(dt);
}
#endif
/**
* Call tgModelVisitor::render() on self and all descendants.
* @param[in,out] r a reference to a tgModelVisitor
*/
void tgCordeModel::onVisit(const tgModelVisitor& r) const
{
    r.render(*this);
}

 // Called from controller class, it makes the restLength get closer to preferredlength.
void tgCordeModel::moveMotors(double dt)
{
	
}

// @todo look into a base class implementation of this. Wouldn't be
// difficult with existing get functions
void tgCordeModel::tensionMinLengthController(const double targetTension,
										float dt)
{
	
}

void tgCordeModel::setRestLength(double newLength, float dt)
{
	
}
  
const double tgCordeModel::getStartLength() const
{
	
}

const double tgCordeModel::getCurrentLength() const 
{
	
}

const double tgCordeModel::getTension() const
{
	
}

const double tgCordeModel::getRestLength() const
{
	
}

const double tgCordeModel::getVelocity() const
{
	
}
