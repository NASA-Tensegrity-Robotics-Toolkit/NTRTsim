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
#include <vector>
#include <set>
#include <map>

// Forward references
class tgWorld;
class muscleAnchor;
class btRigidBody;
class btCollisionShape;
class btPairCachingGhostObject;
class btDynamicsWorld;
class btBroadphaseInterface;
class btDispatcher;
class btPersistentManifold;

class MuscleNP : public Muscle2P
{
public:

    MuscleNP(btPairCachingGhostObject* ghostObject,
            tgWorld& world,
         btRigidBody * body1,
         btVector3 pos1,
         btRigidBody * body2,
         btVector3 pos2,
         double coefK,
         double dampingCoefficient);
         
   virtual ~MuscleNP();
    
    virtual const btScalar getActualLength() const;
    
    ///@todo change this to update(dt) or similar, since that's the role its serving
    virtual btVector3 calculateAndApplyForce(double dt);
    
private:
  
    struct anchorCompare
    {
        anchorCompare(const muscleAnchor* m1, const muscleAnchor* m2);
        
        bool operator() (const muscleAnchor* lhs, const muscleAnchor* rhs) const;

#if (0)        
        void findPosition(std::vector<const muscleAnchor*> anchorList, std::vector<const muscleAnchor*>::Iterator it, const muscleAnchor* anc) const;
#endif        
        bool compareAnchors(const muscleAnchor* lhs, const muscleAnchor* rhs) const;
        
        private:
           const muscleAnchor* const ma1;
           const muscleAnchor* const ma2;
    };
    
    void updateAnchorList();
    
    void pruneAnchors();
    
    void updateCollisionObject();
    
    void deleteCollisionShape(btCollisionShape* pShape);
    
    // Returns whether the anchor was deleteable
    bool deleteAnchor(int i);
    
    std::set<const btPersistentManifold*> m_contactManifolds;
    
    std::pair<std::set<const btPersistentManifold*>::iterator,bool> m_contactCheck;
    
    std::vector<muscleAnchor*>::iterator m_anchorIt;
    
    std::map<btRigidBody*, btVector3> m_rbForceMap;
    std::map<btRigidBody*, btScalar> m_rbForceScales;
    
protected:  
    
  
   btPairCachingGhostObject* m_ghostObject;
   
   tgWorld&  m_world;
   
   btBroadphaseInterface* const m_overlappingPairCache;
  
  btDispatcher* const m_dispatcher;
  
   const anchorCompare m_ac;

    
   
};

#endif  // NTRT_MUSCLENP_H_
