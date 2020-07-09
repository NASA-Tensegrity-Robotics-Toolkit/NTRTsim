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
 * @file tgUnidirComprSprActuatorInfo.h
 * @brief Definition of class tgUnidirComprSprActuatorInfo, 
 * used to create a tgBulletUnidirComprSpr
 * @author Drew Sabelhaus, Ryan Adams
 * @date August 2016
 * $Id$
 */

#ifndef SRC_TGCREATOR_TG_UNIDIR_COMPR_SPR_ACTUATOR_INFO_H
#define SRC_TGCREATOR_TG_UNIDIR_COMPR_SPR_ACTUATOR_INFO_H

// This class inherits from tgCompressionSpringActuatorInfo
#include "tgCompressionSpringActuatorInfo.h"

// From the C++ standard library:
#include <string>

// By including the superclass, tgTags etc. are also included.
#include "core/tgUnidirComprSprActuator.h"

/* Forward declarations, needed in order to create
 * the actual compression spring object and also to 
 * reference its config.
 * TO-DO: why not just include tgBulletUnidirComprSpr.h? */
class tgBulletUnidirComprSpr;

class tgUnidirComprSprActuatorInfo : public tgCompressionSpringActuatorInfo
{
public:

    /**
     * Construct a tgUnidirComprSprActuatorInfo with just a config.
     * The pair must be filled in 
     * later, or factory methods can be used to create instances with
     * pairs.
     */
    tgUnidirComprSprActuatorInfo(
	          const tgUnidirComprSprActuator::Config& config);

    /**
     * Construct a tgUnidirComprSprActuatorInfo with just a config
     *  and tags. The pair must 
     * be filled in later, or factory methods can be used to create instances 
     * with pairs.
     */
    tgUnidirComprSprActuatorInfo(
                   const tgUnidirComprSprActuator::Config& config,
		   tgTags tags);

    /**
     * Construct a tgUnidirComprSprActuatorInfo from its endpoints.
     * @param[in] from one endpoint
     * @param[in] to the other endpoint
     * @param[in] config contains... something else that's needed? TO-DO: why?
     * @todo: make sure that tgPairs returns references to the vectors...
     */
    tgUnidirComprSprActuatorInfo(
		   const tgUnidirComprSprActuator::Config& config,
		   const tgPair& pair);
    
    /**
     * virtual destructor. does nothing.
     */
    virtual ~tgUnidirComprSprActuatorInfo() {}

    /**
     * Create a tgConnectorInfo* from a tgPair.
     */ 
    virtual tgConnectorInfo* createConnectorInfo(const tgPair& pair);

    /** 
     * create the actual tgBulletUnidirComprSpr
     * Calls createTgBulletUnidirComprSpr helper function.
     */
    virtual void initConnector(tgWorld& world);

    /**
     * Return the tgUnidirComprSprActuator that's been built
     * from the tgBulletUnidirComprSpr.
     */
    virtual tgModel* createModel(tgWorld& world);

protected:    

    /**
     * Helper function: actually creates the tgBulletUnidirCompSpr.
     * Note that the m_bulletCompressionSpring is stored in the parent class.
     */
    tgBulletUnidirComprSpr* createTgBulletUnidirComprSpr();

    /**
     * Auxiliary helper to the constructors. This makes it easy to do
     * the same checks in multiple constructors without copy-pasting code.
     */
    void constructorAux();
    
private:

    // local copy of the config struct.
    tgUnidirComprSprActuator::Config m_config;
    
};


#endif
