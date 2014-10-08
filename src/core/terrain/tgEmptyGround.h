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

#ifndef TG_EMPTY_GROUND_H
#define TG_EMPTY_GROUND_H

/**
 * @file tgEmptyGround.h
 * @brief Contains the definition of class tgEmptyGround.
 * @author Brian Mirletz
 * $Id$
 */

#include "tgBulletGround.h"

// Forward declarations
class btRigidBody;

/**
 * An empty ground implementation. tgWorldBulletPhysicsImpl checks if
 * this class is present with a tgCast call, and skips calling
 * getGroundRigidBody if it is.
 */
class tgEmptyGround : public tgBulletGround
{
public:
    
    /**
     * Default construction that uses the default values of config
     * Does nothing
     */
    tgEmptyGround();
    
    /** Clean up the implementation. The base class holds nothing. */
    virtual ~tgEmptyGround() { }
    
    /**
     * This should never be called. Will fail an assertation
     * @todo consider making it an exception instead.
     */
    virtual btRigidBody* getGroundRigidBody() const;

};


#endif  // TG_EMPTY_GROUND_H
