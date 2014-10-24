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

#ifndef NTRT_MUSCLENP_H_
#define NTRT_MUSCLENP_H_

/**
 * @file MuscleNP.h
 * @brief Definition of a massless cable with contact dynamics
 * $Id$
 */

// NTRT
#include "core/Muscle2P.h"
// The Bullet Physics library
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
// The C++ Standard Library

#include <string>
#include <set>

// Forward references
class muscleAnchor;
class btRigidBody;
class btPairCachingGhostObject;
class btBroadphaseInterface;

class MuscleNP : public Muscle2P
{
public:

    MuscleNP(btPairCachingGhostObject* ghostObject,
            btBroadphaseInterface* broadphase,
         btRigidBody * body1,
         btVector3 pos1,
         btRigidBody * body2,
         btVector3 pos2,
         double coefK,
         double dampingCoefficient);
         
   virtual ~MuscleNP();
    
    ///@todo change this to update(dt) or similar, since that's the role its serving
    virtual btVector3 calculateAndApplyForce(double dt);
    
private:
  
    struct anchorCompare
    {
        anchorCompare(const muscleAnchor* m1, const muscleAnchor* m2);
        
        bool operator() (const muscleAnchor* lhs, const muscleAnchor* rhs);
        
        
        private:
           const muscleAnchor* const ma1;
           const muscleAnchor* const ma2;
    };
    
    void updateAnchorList(double dt);
    
protected:    
   btPairCachingGhostObject* m_ghostObject;
   
   btBroadphaseInterface*   m_overlappingPairCache;
   
   std::set<const muscleAnchor*> m_anchors;
  
   anchorCompare m_ac;

    
   
};

#endif  // NTRT_MUSCLENP_H_
