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
 * @file tgRigidAutoCompound.h
 * @brief Definition of class tgRigidAutoCompound
 * @author Ryan Adams
 * @date March 2014
 * $Id$
 */

#ifndef TG_RIGID_AUTO_COMPOUND_H
#define TG_RIGID_AUTO_COMPOUND_H

#include <vector>
#include <deque>

class tgRigidInfo;
class btCollisionObject;
class btRigidBody;

/**
 * Rigids that share nodes should automatically be compounded before initializing.
 * This automatically compounds shapes that rigids that share nodes while maintaining
 * the original mapping for later transformation back into the original list (but with
 * the RigidBody being a compound object that multiple tgRigidInfos may point to).
 */
class tgRigidAutoCompound {
public:

public:       
    // @todo: we want to start using this and get rid of the set-based constructor, but until we can refactor...
    tgRigidAutoCompound(std::vector<tgRigidInfo*> rigids);
    
    tgRigidAutoCompound(std::deque<tgRigidInfo*> rigids);
    
    ~tgRigidAutoCompound()
    {
    }
    
    std::vector< tgRigidInfo* > execute();

protected:
    
    // @todo: we probably don't need this any more -- this will be taken care of in the tgRigidInfo => tgModel step
    // @todo: NOTE: we need to have a way to check to see if a rigid has already been instantiated -- maybe just check get
    void setRigidBodyForGroup(btCollisionObject* body, std::deque<tgRigidInfo*>& group);
   
    void setRigidInfoForGroup(tgRigidInfo* rigidInfo, std::deque<tgRigidInfo*>& group);
    
    void groupRigids();

    // Find all rigids that should be in a group with the given rigid
    // @todo: This may contain an off-by-one error (the last rigid may not be grouped properly...)
    std::deque<tgRigidInfo*> findGroup(tgRigidInfo* rigid, std::deque<tgRigidInfo*>& ungrouped);
        
    void createCompounds();
    
    tgRigidInfo* createCompound(std::deque<tgRigidInfo*> rigids);
    
    bool rigidBelongsIn(tgRigidInfo* rigid, std::deque<tgRigidInfo*> group);
    
    // Doesn't look like we own these
    std::deque<tgRigidInfo*> m_rigids;
    std::vector< std::deque<tgRigidInfo*> > m_groups;
    std::vector< tgRigidInfo* > m_compounded;  // temporary set of compounded rigids. Same keys as m_groups

};



#endif
