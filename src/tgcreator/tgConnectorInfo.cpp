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
 * @file tgConnectorInfo.cpp
 * @brief Implementation of class tgConnectorInfo
 * @author Ryan Adams
 * @date January 2014
 * $Id$
 */

#include "tgConnectorInfo.h"

#include "tgPair.h"
#include "tgPairs.h"
#include "tgRigidInfo.h"

#include "core/tgTagSearch.h"

#include "LinearMath/btVector3.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"


tgConnectorInfo* tgConnectorInfo::createConnectorInfo(const tgPair& pair, const tgTagSearch& tagSearch)
{
    // Our subclasses may not be able to create connectorInfos based on nodes. Also, the
    // tags may not match our search. Make sure both work, or return 0. 
    tgConnectorInfo* connectorInfo = 0;
    if(tagSearch.matches(pair.getTags())) {
        connectorInfo = createConnectorInfo(pair);
    }
    return connectorInfo;
}

std::vector<tgConnectorInfo*> tgConnectorInfo::createConnectorInfos(const tgPairs& pairs, const tgTagSearch& tagSearch)
{
    std::vector<tgConnectorInfo*> result;
    for(int i = 0; i < pairs.size(); i++) {
        tgConnectorInfo* r = createConnectorInfo(pairs[i], tagSearch);
        if(r != 0) {
            result.push_back(r);
        }
    }
    return result;
}

void tgConnectorInfo::chooseRigids(std::set<tgRigidInfo*> rigids) 
{

    // @todo: find and set pointers to appropriate rigids from the set provided. 
    // @todo: should we throw an exception if no appropriate rigid is found? 
    if(getFromRigidInfo() == 0) { // if it hasn't already been set
        tgRigidInfo* fromRigidInfo = chooseRigid(rigids, getFrom());
        //std::cout << "  chosen fromRigidInfo is " << fromRigidInfo << std::endl;
        setFromRigidInfo(fromRigidInfo);
        //std::cout << "  getFromRigidInfo is " << getFromRigidInfo() << std::endl;
    }
    
    if(getToRigidInfo() == 0) { // if it hasn't already been set
        tgRigidInfo* toRigidInfo = chooseRigid(rigids, getTo());
        //std::cout << "  chosen toRigidInfo is " << toRigidInfo << std::endl;
        setToRigidInfo(toRigidInfo);
        //std::cout << "  getToRigidInfo is " << getToRigidInfo() << std::endl;
    }
}

tgRigidInfo* tgConnectorInfo::chooseRigid(std::set<tgRigidInfo*> rigids, const btVector3& v) {

    std::set<tgRigidInfo*> candidateRigids = findRigidsContaining(rigids, v);
    
    tgRigidInfo* chosenRigid;
    if (candidateRigids.size() == 1) {
        // Choose the first element since there's only one
        chosenRigid = *(candidateRigids.begin());  
    } else {
        // find the best candidate (if more than one rigid, use the rigid whose center of mass is closest to v. This seems like a reasonable approach...)
        chosenRigid = findClosestCenterOfMass(candidateRigids, v);
    }

    return chosenRigid;
};

btRigidBody* tgConnectorInfo::getToRigidBody() {
    return getToRigidInfo()->getRigidInfoGroup()->getRigidBody();
    //return m_toRigidBody;
};

btRigidBody* tgConnectorInfo::getFromRigidBody() {
    //return m_fromRigidBody;
    return getFromRigidInfo()->getRigidInfoGroup()->getRigidBody();
};


// Protected:


tgRigidInfo* tgConnectorInfo::findClosestCenterOfMass(std::set<tgRigidInfo*> rigids, const btVector3& v) {
    if (rigids.size() == 0) {
        return NULL;
    }
    std::set<tgRigidInfo*>::iterator it;
    it = rigids.begin();
    tgRigidInfo* closest = *it;  // First member
    it++;
    for(; it != rigids.end(); ++it) {
        btVector3 com = (*it)->getCenterOfMass();
        if(com.distance(v) < closest->getCenterOfMass().distance(v)) {
            closest = *it;
        }
    }
    return closest;
}


std::set<tgRigidInfo*> tgConnectorInfo::findRigidsContaining(std::set<tgRigidInfo*> rigids, const btVector3& toFind) {
    std::set<tgRigidInfo*> found;
    std::set<tgRigidInfo*>::iterator it;
    for(it=rigids.begin(); it != rigids.end(); ++it) {
        if ((*it)->containsNode(toFind)) {
            found.insert(*it);
        }
    }
    return found;
};

// @todo: Remove this? Is it used by anything? It's protected...
bool tgConnectorInfo::rigidFoundIn(std::set<tgRigidInfo*> rigids, tgRigidInfo* rigid) {
    //return (std::find(rigids.begin(), rigids.end(), rigid) != rigids.end()); // Doesn't work on some compilers (RDA 2014-Jan-28)
    std::set<tgRigidInfo*>::iterator it;
    for(it = rigids.begin(); it != rigids.end(); ++it) {
        if(*it == rigid) 
            return true;
    }
    return false;
};    
