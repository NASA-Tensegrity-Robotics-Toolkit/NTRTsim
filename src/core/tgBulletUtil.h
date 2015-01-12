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

#ifndef TG_BULLET_UTIL_H
#define TG_BULLET_UTIL_H

/**
 * @file tgBulletUtil.h
 * @brief Contains the definition of class tgBulletUtil.
 * @date January 21, 2014
 * $Id$
 */

// Forward declarations
class btCollisionShape;
class btDynamicsWorld;
class btRigidBody;
class btTransform;
class tgWorld;

/**
 * Utility class for dealing with Bullet Physics
 */
class tgBulletUtil
{
public:

    // @todo: Move this to the tgRigidInfo => tgModel step
    // NOTE: this is a copy of localCreateRigidBody from the bullet DemoApplication. 
    static btRigidBody* createRigidBody(btDynamicsWorld* dynamicsWorld, 
                                        float mass, 
                                        const btTransform& startTransform, 
                                        btCollisionShape* shape);
    /**
     * Assuming that world has a tgWorldBulletPhysicsImpl, return
     * its dynamics world.
     * @param[in,out] world a tgWorld
     * @return the world's implementation's btSoftRigidDynamicsView
     * @todo consider implications of casting to include Corde objects
     */
    static btDynamicsWorld& worldToDynamicsWorld(const tgWorld& world);
};


#endif
