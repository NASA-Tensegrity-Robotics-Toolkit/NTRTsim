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

#ifndef TG_BULLET_GROUND_H
#define TG_BULLET_GROUND_H

/**
 * @file tgBulletGround.h
 * @brief Contains the definition of class tgBulletGround
 * @author Brian Tietz
 * $Id$
 */

#include "tgGround.h"

// Forward declarations
class btRigidBody;
class btCollisionShape;

/**
 * Abstract base class that defines the parameters required for ground
 * used by Bullet Physics Implementations
 */
class tgBulletGround : public tgGround
{
public:

    /** 
    * The only constructor. The base class initializes nothing.
    * @param[in] config configuration POD
    */
    tgBulletGround();

    /** Clean up the implementation. Deletes the collision object */
    virtual ~tgBulletGround();
    
    /** Returns the rigid body to the bullet physics implementation */
    virtual btRigidBody* getGroundRigidBody() const = 0;
	
	/** 
	 * Returns a pointer to the collision shape for the list of 
	 * collision objects in tgWorldBulletPhysicsImpl
	 * Collision shape must not be null
	 */
    btCollisionShape* const getCollisionShape() const;    

protected:
    // Will take care of deleting this ourselves.
    btCollisionShape* pGroundShape;
};


#endif  // TG_WORLDIMPL_H
