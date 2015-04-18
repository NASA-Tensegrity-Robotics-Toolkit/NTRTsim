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

#ifndef TG_CONNECTOR_INFO_H
#define TG_CONNECTOR_INFO_H

/**
 * @file tgConnectorInfo.h
 * @brief Definition of class tgConnectorInfo
 * @author Ryan Adams
 * @date January 2014
 * $Id$
 */

#include "core/tgTaggable.h"

class btVector3;
class tgNode;
class tgPair;
class tgPairs;
class tgTagSearch;
class tgRigidInfo;
class btRigidBody;
class tgModel;
class tgWorld;

#include "LinearMath/btVector3.h" // @todo: any way to move this to the .cpp file?
#include "tgPair.h"

class tgConnectorInfo : public tgTaggable {
public:

    tgConnectorInfo() : 
        tgTaggable(),
        m_fromRigidInfo(0),
        m_toRigidInfo(0)
        //m_fromRigidBody(0),
        //m_toRigidBody(0),
        
    {
        // Supress compiler warning for bullet's unused variable
        (void) btInfinityMask;
    }    

    tgConnectorInfo(tgTags tags) : 
        tgTaggable(tags),
        m_fromRigidInfo(0),
        m_toRigidInfo(0)
        //m_fromRigidBody(0),
        //m_toRigidBody(0),
    {}    

    tgConnectorInfo(const std::string& space_separated_tags) :
        tgTaggable(space_separated_tags),
        m_fromRigidInfo(0),
        m_toRigidInfo(0)
        //m_fromRigidBody(0),
        //m_toRigidBody(0),
    {}    

    tgConnectorInfo(const tgPair& pair) :
        tgTaggable(pair.getTags()),
        m_pair(pair),
        m_fromRigidInfo(0),
        m_toRigidInfo(0)
        //m_fromRigidBody(0),
        //m_toRigidBody(0),
    {}    


    virtual ~tgConnectorInfo() {};


    virtual tgConnectorInfo* createConnectorInfo(const tgPair& pair) = 0;
    
    virtual tgConnectorInfo* createConnectorInfo(const tgPair& pair, const tgTagSearch& tagSearch);

    virtual std::vector<tgConnectorInfo*> createConnectorInfos(const tgPairs& pairs, const tgTagSearch& tagSearch);
    
    virtual void initConnector(tgWorld& world) = 0;
    
    virtual tgModel* createModel(tgWorld& world) = 0;
    

    btVector3& getFrom() {
        return m_pair.getFrom();
    }; 
    const btVector3& getFrom() const 
    {
        return m_pair.getFrom();
    }; 

    btVector3& getTo() {
        return m_pair.getTo();
    };
    const btVector3& getTo() const {
        return m_pair.getTo();
    };
        
    tgRigidInfo* getFromRigidInfo() {
        return m_fromRigidInfo;
    };
    const tgRigidInfo* getFromRigidInfo() const {
        return m_fromRigidInfo;
    };
    
    void setFromRigidInfo(tgRigidInfo* rigidInfo)
    {
        m_fromRigidInfo = rigidInfo;
    }
    
    tgRigidInfo* getToRigidInfo() {
        return m_toRigidInfo;
    };
    const tgRigidInfo* getToRigidInfo() const {
        return m_toRigidInfo;
    };

    void setToRigidInfo(tgRigidInfo* rigidInfo)
    {
        m_toRigidInfo = rigidInfo;
    }

    // @todo: Do we want to have these separate or use the rigidBody from the rigidInfos??
    btRigidBody* getToRigidBody();

    btRigidBody* getFromRigidBody();
    
    // @todo: how should we calculate mass? 
    // Note that different connectors will likely use different methods of calculating mass...
    virtual double getMass() = 0;
    
    
    // Choose the appropriate rigids for the connector and give the connector pointers to them
    virtual void chooseRigids(std::set<tgRigidInfo*> rigids);

    // @todo: in the process of switching ti std::vector for these...
    virtual void chooseRigids(std::vector<tgRigidInfo*> rigids) 
    {
        std::set<tgRigidInfo*> s;
        s.insert(rigids.begin(), rigids.end());
        chooseRigids(s);
    }

    
    tgRigidInfo* chooseRigid(std::set<tgRigidInfo*> rigids, const btVector3& v);
    
    
protected:
    tgRigidInfo* findClosestCenterOfMass(std::set<tgRigidInfo*> rigids, const btVector3& v);

    // @todo: should this be protected/private?
    std::set<tgRigidInfo*> findRigidsContaining(std::set<tgRigidInfo*> rigids, const btVector3& toFind);
    
    // @todo: Remove this? Is it used by anything?
    bool rigidFoundIn(std::set<tgRigidInfo*> rigids, tgRigidInfo* rigid);
    
    
    // Step 1: Define the points that we're connecting
    tgPair m_pair;
    
    // Step 2: Using the points from step 1, the builders will find the right tgRigidInfo objects 
    tgRigidInfo* m_fromRigidInfo;
    tgRigidInfo* m_toRigidInfo;

};

/**
 * Overload operator<<() to handle tgConnectorInfo
 * @param[in,out] os an ostream
 * @param[in] a tgConnectorInfo
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 */
inline std::ostream&
operator<<(std::ostream& os, const tgConnectorInfo& n)
{

    os << "tgConnectorInfo(";
    os <<  n.getFrom() << ", ";
    os <<  n.getTo() << ", ";
    os << "fromRigidInfo: " << n.getFromRigidInfo() << ", ";
    os << "toRigidInfo: " << n.getToRigidInfo() << ", ";
    os << "Tags: " << n.getTags();
    os << ")";

    return os;
}

#endif
