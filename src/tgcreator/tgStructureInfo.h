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
 * @file tgStructureInfo.h
 * @brief Definition of class tgStructureInfo
 * @author Ryan Adams
 * @date March 2014
 * $Id$
 */

#ifndef TG_STRUCTURE_INFO_H
#define TG_STRUCTURE_INFO_H

// This library
#include "tgBuildSpec.h"
// NTRT Core library
#include "core/tgTaggable.h"
// The C++ Standard Library
#include <iostream>
#include <vector>

// Forward declarations
class tgBuildSpec;
class tgConnectorInfo;
class tgModel;
class tgRigidInfo;
class tgStructure;
class tgWorld;

/**
 * Representation of a structure containing all info required to build it into
 * a model or extract data for analysis (e.g. a matrix of connections, etc.)
 */
class tgStructureInfo : public tgTaggable
{

    friend std::ostream& operator<<(std::ostream& os, const tgStructureInfo& obj);

public:

    tgStructureInfo(tgStructure& structure, tgBuildSpec& buildSpec);

    tgStructureInfo(tgStructure& structure, tgBuildSpec& buildSpec, const tgTags& tags);

    virtual ~tgStructureInfo();
    
    // Return all rigids in this structure and its descendants
    std::vector<tgRigidInfo*> getAllRigids() const;
    
    const std::vector<tgConnectorInfo*>& getConnectors() const
    {
        return m_connectors;
    }

    // Build our info into the provided model
    void buildInto(tgModel& model, tgWorld& world);

private:

    /*
     * Initialize all the rigidInfo and connectorInfo objects for this structureInfo and all of its children
     */
    void addRigidsAndConnectors();

    /*
     * Create and return a rigidInfo object using a matching rigidAgent
     */
    template <class T>
    tgRigidInfo* initRigidInfo(const T& rigidCandidate, const std::vector<tgBuildSpec::RigidAgent*>& rigidAgents) const;

    /*
     * Create and return a connectorInfo object using a matching connectorAgent
     */
    template <class T>
    tgConnectorInfo* initConnectorInfo(const T& connectorCandidate, const std::vector<tgBuildSpec::ConnectorAgent*>& connectorAgents) const;

    void autoCompoundRigids();
    
    void chooseConnectorRigids();

    void chooseConnectorRigids(std::vector<tgRigidInfo*> allRigids);
    
    void initRigidBodies(tgWorld& world);
    
    void initConnectors(tgWorld& world);
    
    const std::vector<tgRigidInfo*>& getRigids() const
    {
        return m_rigids;
    }
    
    void addChild(tgStructureInfo* pChild);
    
    const std::vector<tgStructureInfo*>& getChildren() const
    {
        return m_children;
    }
    
    /**
     * Create our own tree to mirror the structure (StructureInfo only)
     */
    void createTree(tgStructureInfo& scaffold, const tgStructure& structure);

    void buildIntoHelper(tgModel& model, tgWorld& world, tgStructureInfo& structureInfo);
    
    std::string toString(const std::string& prefix = "") const;
    
private:

    tgStructure& m_structure;

    tgBuildSpec& m_buildSpec;
    
    // We do own these
    std::vector<tgRigidInfo*> m_rigids;

    std::vector<tgConnectorInfo*> m_connectors;

    std::vector<tgStructureInfo*> m_children;
    
    std::vector<tgRigidInfo*> m_compounded;
};

/**
 * Overload operator<<() to handle tgStructureInfo
 * @param[in,out] os an ostream
 * @param[in] a reference to a tgStructureInfo
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 */
std::ostream& operator<<(std::ostream& os, const tgStructureInfo& obj);

#endif
