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
 * @file tgCompressionSpringActuatorInfo.h
 * @brief Definition of class tgCompressionSpringActuatorInfo, used to create a tgBulletCompressionSpring
 * @author Drew Sabelhaus, et al.
 * @date August 2016
 * $Id$
 */

#ifndef SRC_TGCREATOR_TG_COMPRESSION_SPRING_ACTUATOR_INFO_H
#define SRC_TGCREATOR_TG_COMPRESSION_SPRING_ACTUATOR_INFO_H

/* Includes from the tgcreator folder*/
#include "tgConnectorInfo.h"

#include "tgRigidInfo.h"

#include <string>


#include "core/tgCompressionSpringActuator.h"
#include "core/tgTags.h"

/* Forward declarations, needed in order to create
 * the actual compression spring object and also to 
 * reference its config.
 * TO-DO: why not just include tgBulletCompressionSpring.h? */
class tgBulletCompressionSpring;

// The compression spring actuator serves as a connector
class tgCompressionSpringActuatorInfo : public tgConnectorInfo
{
public:

    /**
     * Construct a tgCompressionSpringActuatorInfo with just a config. 
     * The pair must be filled in later, or factory methods can 
     * be used to create instances with pairs.
     */
    tgCompressionSpringActuatorInfo(const tgCompressionSpringActuator::Config& config);

    /**
     * Construct a tgCompressionSpringActuatorInfo with just a config and tags.
     * The pair must be filled in later, or factory methods can 
     * be used to create instances with pairs.
     */
    tgCompressionSpringActuatorInfo(const tgCompressionSpringActuator::Config& config, tgTags tags);

    /**
     * Construct a tgCompressionSpringActuatorInfo from its endpoints.
     * @param[in] from one endpoint
     * @param[in] to the other endpoint
     * @param[in] config contains... something else that's needed? TO-DO: why?
     * @todo: make sure that tgPairs returns references to the vectors...
     */
    tgCompressionSpringActuatorInfo(const tgCompressionSpringActuator::Config& config, const tgPair& pair);
    
    /**
     * virtual destructor, does nothing.
     */
    virtual ~tgCompressionSpringActuatorInfo() {}
    
    /**
     * Create a tgConnectorInfo* from a tgPair.
     */ 
    virtual tgConnectorInfo* createConnectorInfo(const tgPair& pair);

    /** 
     * create the actual tgBulletCompressionSpring
     * Calls createTgBulletCompressionSpring helper function.
     */
    virtual void initConnector(tgWorld& world);

    /**
     * Return the tgCompressionSpringActuator that's been built
     * from the tgBulletCompressionSpring.
     */
    virtual tgModel* createModel(tgWorld& world);

    double getMass();

protected:    

    /**
     * Helper function: actually creates the tgBulletCompressionSpring.
     */
    tgBulletCompressionSpring* createTgBulletCompressionSpring();

    /**
     * reference to the tgBulletCompressionSpring that's created by the above method.
     */
    tgBulletCompressionSpring* m_bulletCompressionSpring;
    
private:
    
    tgCompressionSpringActuator::Config m_config;
    
};


#endif
