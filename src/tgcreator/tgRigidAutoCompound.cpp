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
 * @file tgRigidAutoCompound.cpp
 * @brief Definition of class tgRigidAutoCompound
 * @author Ryan Adams
 * @date March 2014
 * $Id$
 */

#include "tgRigidAutoCompound.h"

#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "tgCompoundRigidInfo.h"
#include <map>

// Debugging
#include <iostream>
#include "tgUtil.h"

using namespace std;

    
// @todo: we want to start using this and get rid of the set-based constructor, but until we can refactor...
tgRigidAutoCompound::tgRigidAutoCompound(std::vector<tgRigidInfo*> rigids)
{
    m_rigids.insert(m_rigids.end(), rigids.begin(), rigids.end());
}

tgRigidAutoCompound::tgRigidAutoCompound(std::deque<tgRigidInfo*> rigids) : m_rigids(rigids)
{}
    
std::vector< tgRigidInfo* > tgRigidAutoCompound::execute() {

    // Determine the grouping of our rigids
    groupRigids();

    // Create the compounds as necessary
    createCompounds();

    // Set the rigid body for the various groups
    for(int i=0; i < m_groups.size(); i++) {
        // Note: rigids that are not connected to anything else are placed
        //into a group of their own, so they're represented here too
        setRigidInfoForGroup(m_compounded[i], m_groups[i]);
    }
    // Need to return this so we can delete it at the appropreate time
    return m_compounded;
};
       
// @todo: we probably don't need this any more -- this will be taken care of in the tgRigidInfo => tgModel step
// @todo: NOTE: we need to have a way to check to see if a rigid has already been instantiated -- maybe just check get
void tgRigidAutoCompound::setRigidBodyForGroup(btCollisionObject* body, std::deque<tgRigidInfo*>& group) {
    for(int i = 0; i < group.size(); i++) {
        group[i]->setCollisionObject(body);
    }
}

void tgRigidAutoCompound::setRigidInfoForGroup(tgRigidInfo* rigidInfo, std::deque<tgRigidInfo*>& group) {
    for(int i = 0; i < group.size(); i++) {
        group[i]->setRigidInfoGroup(rigidInfo);
    }
}

void tgRigidAutoCompound::groupRigids()
{
    std::deque<tgRigidInfo*> ungrouped = std::deque<tgRigidInfo*>(m_rigids); // Copy of m_rigids

    while(ungrouped.size() > 0) {
        // go through each ungrouped element and find the groups for it
        tgRigidInfo* elem = ungrouped[0]; // Note: the 0 element is removed by findGroup, so ungrouped[0] is different each iteration

        // Find all elements linked to the first item in ungrouped
        // This will also remove the elements in the group from ungrouped
        std::deque<tgRigidInfo*> group = findGroup(elem, ungrouped);

        // Add the group to the groups list
        m_groups.push_back(group);            
    }
}

// Find all rigids that should be in a group with the given rigid
// @todo: This may contain an off-by-one error (the last rigid may not be grouped properly...)
std::deque<tgRigidInfo*> tgRigidAutoCompound::findGroup(tgRigidInfo* rigid, std::deque<tgRigidInfo*>& ungrouped) {

    std::deque<tgRigidInfo*> group;

    // Add the rigid to the current group
    group.push_back(rigid);     
    
    // Remove the rigid from the ungrouped since it's now in a group
    ungrouped.erase(std::remove(ungrouped.begin(), ungrouped.end(), rigid), ungrouped.end());
    
    // Recursively find linked elements
    int i = 0;
    while(i < ungrouped.size()) {
        tgRigidInfo* other = ungrouped[i];
        if(rigid->sharesNodesWith(*other)) {
            std::deque<tgRigidInfo*> links = findGroup(other, ungrouped);
            group.insert(group.end(), links.begin(), links.end());
            i = 0;
        } else {
            i++;
        }
    }

    return group;
};
    
void tgRigidAutoCompound::createCompounds() {
    for(int i=0; i < m_groups.size(); i++) {
        std::deque<tgRigidInfo*>& group = m_groups[i];
        if(group.size() == 1) {
            // Only one, no compounding necessary
            m_compounded.push_back(group[0]);
        } else {
            tgRigidInfo* compound = createCompound(group);
            m_compounded.push_back(compound);
        }
    }
}

tgRigidInfo* tgRigidAutoCompound::createCompound(std::deque<tgRigidInfo*> rigids) {
    tgCompoundRigidInfo* c = new tgCompoundRigidInfo();
    for(int i = 0; i < rigids.size(); i++) {
        c->addRigid(*rigids[i]);
    }
    return (tgRigidInfo*)c;
}

bool tgRigidAutoCompound::rigidBelongsIn(tgRigidInfo* rigid, std::deque<tgRigidInfo*> group) {
    for(int i = 0; i < group.size(); i++) {
        tgRigidInfo* other = group[i];
        if(rigid->sharesNodesWith(*other))
            return true;
    }
    return false;
};
