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

#ifndef NTRT_MUSCLE_ANCHOR_H_
#define NTRT_MUSCLE_ANCHOR_H_

/**
 * @file MuscleAnchor.h
 * @brief Definitions of class MuscleAnchor.
 * $Id$
 */

// The Bullet Physics library
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
// The C++ Standard Library

#include <string>

class btRigidBody;

class muscleAnchor
{
public:
   

    muscleAnchor(btRigidBody *body, btVector3 pos);
    
    ~muscleAnchor();
    
    btVector3 getWorldPosition();

    // Relative to the body
    btVector3 getRelativePosition();

    btRigidBody * attachedBody;

    // Relative to the body when it is first constructed
    btVector3 attachedRelativeOriginalPosition;

    btScalar height;
 private:
  muscleAnchor();
};

#endif //NTRT_MUSCLE_ANCHOR_H_
