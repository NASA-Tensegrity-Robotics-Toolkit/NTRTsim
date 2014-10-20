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

#ifndef TG_CRATER_GROUND_H
#define TG_CRATER_GROUND_H

/**
 * @file tgCraterGround.h
 * @brief Contains the definition of class tgCraterGround.
 * @author Steven Lessard
 * $Id$
 */

#include "tgBulletGround.h"

#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
// Forward declarations
class btRigidBody;

/**
 * Simplest ground implementation - a flat box. Can be put at an angle
 * to allow for an incline
 */
class tgCraterGround : public tgBulletGround
{
public:
    
    struct Config
    {
    public:
        Config( btVector3 eulerAngles = btVector3(0.0, 0.0, 0.0),
                btScalar friction = 0.5,
                btScalar restitution = 0.0,
                btVector3 size = btVector3(500.0, 1.5, 500.0),
                btVector3 origin = btVector3(0.0, 0.0, 0.0) );
      /**
       * Euler angles are specified as yaw pitch and roll
       */
      btVector3 m_eulerAngles;
      
      /**
       * Friction value of the ground, must be between 0 to 1 
       */
      btScalar  m_friction;
      
       /**
       * Restitution coefficient of the ground, must be between 0 to 1 
       */
      btScalar  m_restitution;
      
      /**
       * Size of the ground, must be between non-negitive
       */
      btVector3 m_size;
      
      /**
       * Origin position of the ground
       */
      btVector3 m_origin;
    };
    
    /**
     * Default construction that uses the default values of config
     * Sets up a collision object that is stored in the bulletGround
     * object
     */
    tgCraterGround();
    
    /**
     * Allows a user to specify their own config
     */
    tgCraterGround(const tgCraterGround::Config& config);

    /** Clean up the implementation. The base class holds nothing. */
    virtual ~tgCraterGround() { }
    
    /**
     * Setup and return a return a rigid body based on the collision 
     * object
     */
    virtual btRigidBody* getGroundRigidBody() const;

private:  
    /**
     * Store the configuration data for use later
     */
    Config m_config;
};


#endif  // TG_CRATER_GROUND_H
