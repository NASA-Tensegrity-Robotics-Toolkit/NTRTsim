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

#ifndef TG_CORDE_MODEL
#define TG_CORDE_MODEL

#include "core/tgModel.h"
#include "core/tgBaseString.h"
#include "core/tgSubject.h"

#include "dev/Corde/CordeModel.h"

// Forward Declaration
class cordeCollisionObject;

class tgCordeModel : public tgBaseString, public tgSubject <tgCordeModel>
{
public:
	struct Config
	{
		Config(tgBaseString::Config motor_config, CordeModel::Config string_config);
		
		tgBaseString::Config motorConfig;
		CordeModel::Config stringConfig;
		
	};
	
	/// @todo Consider making CordeModel::config a nested member of a tgCordeModel config (that includes a base string config)
public:
    tgCordeModel(cordeCollisionObject* string, tgBaseString::Config motor_config, const tgTags& tags);
    
    virtual ~tgCordeModel();
    
    virtual void setup(tgWorld& world);
    
    virtual void teardown();
    
    /**
    * Call tgModelVisitor::render() on self and all descendants.
    * @param[in,out] r a reference to a tgModelVisitor
    */
    virtual void onVisit(const tgModelVisitor& r) const;
    
    virtual void step(const double dt);
    
    /** @todo consider adding a toString method **/
    
    const cordeCollisionObject* const getModel() const
    {
        return m_string;
    }
    
        /**
     * Functions for interfacing with muscle2P, and higher level controllers
     */
     
    // Called from controller class, it makes the restLength get closer to preferredlength.
    virtual void moveMotors(double dt);
    
    // @todo look into a base class implementation of this. Wouldn't be
    // difficult with existing get functions
    virtual void tensionMinLengthController(const double targetTension,
                                            float dt);
    
    virtual void setRestLength(double newLength, float dt);
      
    virtual const double getStartLength() const;
    
    virtual const double getCurrentLength() const;
    
    virtual const double getTension() const;
    
    virtual const double getRestLength() const;
    
    virtual const double getVelocity() const;
    
private:
	void logHistory(const double dt);
	
    cordeCollisionObject* m_string;
    
    double m_prevLength;
    

};

#endif //TG_CORDE_MODEL
