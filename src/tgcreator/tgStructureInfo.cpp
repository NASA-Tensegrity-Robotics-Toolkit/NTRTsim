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
 * @file tgStructureInfo.cpp
 * @brief Implementation of class tgStructureInfo 
 * @author Ryan Adams
 * @date January 2014
 * $Id$
 */

// This module
#include "tgStructureInfo.h"
// This library
#include "tgBuildSpec.h"
#include "tgConnectorInfo.h"
#include "tgRigidAutoCompound.h"
#include "tgStructure.h"
#include "core/tgWorld.h"
#include "core/tgModel.h"
// The C++ Standard Library
#include <stdexcept>

tgStructureInfo::tgStructureInfo(tgStructure& structure, tgBuildSpec& buildSpec) : 
    tgTaggable(),
    m_structure(structure), 
    m_buildSpec(buildSpec)
{
    createTree(*this, structure);    
}

tgStructureInfo::tgStructureInfo(tgStructure& structure, tgBuildSpec& buildSpec,
                 const tgTags& tags) :
    tgTaggable(tags),
    m_structure(structure), 
    m_buildSpec(buildSpec)
{
    createTree(*this, structure);    
}

tgStructureInfo::~tgStructureInfo()
{
    // Have to do this first, if all the rigids are deleted it segfaults
    for (std::size_t i = 0; i < m_compounded.size(); i++)
    {
        const tgRigidInfo * const pRigidInfo = m_compounded[i];
    assert(pRigidInfo != NULL);
    // If this is true, rigid is in a group with itself, and 
    // will be deleted with m_rigids
    if (pRigidInfo->getRigidInfoGroup() != pRigidInfo)
    {
        delete pRigidInfo;
    }
    }
    
    for (std::size_t i = 0; i < m_rigids.size(); i++)
    {
    delete m_rigids[i];
    }
    
    for (std::size_t i = 0; i < m_connectors.size(); i++)
    {
    delete m_connectors[i];
    }
    
    for (std::size_t i = 0; i < m_children.size(); i++)
    {
    delete m_children[i];
    }
}

void tgStructureInfo::createTree(tgStructureInfo& structureInfo,
                 const tgStructure& structure)
{
    const std::vector<tgStructure*> children = structure.getChildren();
    for (std::size_t i = 0; i < children.size(); i++)
    {
        tgStructure * const pStructure = children[i];
    assert(pStructure != NULL);
        tgStructureInfo* const pStructureInfo =
        new tgStructureInfo(*pStructure, m_buildSpec, pStructure->getTags());
        structureInfo.addChild(pStructureInfo);
        createTree(*pStructureInfo, *pStructure);
    }
}

std::vector<tgRigidInfo*> tgStructureInfo::getAllRigids() const
{
    std::vector<tgRigidInfo*> result;
    result.insert(result.end(), m_rigids.begin(), m_rigids.end());
 
    // Collect child rigids
    for (std::size_t i = 0; i < m_children.size(); i++)
    {
        tgStructureInfo * const pStructureInfo = m_children[i];
    assert(pStructureInfo != NULL);
        std::vector<tgRigidInfo*> childRigids = pStructureInfo->getAllRigids();
        result.insert(result.end(), childRigids.begin(), childRigids.end());
    }
    
    return result;
}

////////////////////////////
// Build methods
////////////////////////////

void tgStructureInfo::initRigidInfo()
{
    // Step through each of the nodes and pairs, passing them through the rigidAgents to get a list of tgRigidInfos to be added to this. 

    const std::vector<tgBuildSpec::RigidAgent*> rigidAgents = m_buildSpec.getRigidAgents();

    for (std::size_t i = 0; i < rigidAgents.size(); i++)
    {
        tgBuildSpec::RigidAgent * const pRigidAgent = rigidAgents[i];
        assert(pRigidAgent != NULL);

        tgTagSearch tagSearch = tgTagSearch(pRigidAgent->tagSearch);

        // Remove our tags so that subcomponents 'inherit' them (because of the
        // way tags work, removing a tag from the search is the same as adding 
        // the tag to children to be searched)
        tagSearch.remove(getTags());

        tgRigidInfo* pRigidInfo = pRigidAgent->infoFactory;
        assert(pRigidInfo != NULL);

        // Nodes
        std::vector<tgRigidInfo*> nodeRigids =
        pRigidInfo->createRigidInfos(m_structure.getNodes(), tagSearch);
        m_rigids.insert(m_rigids.end(), nodeRigids.begin(), nodeRigids.end());

        // Pairs
        std::vector<tgRigidInfo*> pairRigids =
       pRigidInfo->createRigidInfos(m_structure.getPairs(), tagSearch);
        m_rigids.insert(m_rigids.end(), pairRigids.begin(), pairRigids.end());

    }
    
    // Children
    for (std::size_t i = 0; i < m_children.size(); i++)
    {
        tgStructureInfo* const pStructureInfo = m_children[i];
        
        assert(pStructureInfo != NULL);
        pStructureInfo->initRigidInfo();
    }
}

void tgStructureInfo::autoCompoundRigids()
{
    tgRigidAutoCompound c(getAllRigids());
    m_compounded = c.execute();
}

void tgStructureInfo::initConnectorInfo()
{
    // Step through each of the nodes and pairs, passing them through the
    // rigidAgents to get a list of tgRigidInfos to be added to this. 

    const std::vector<tgBuildSpec::ConnectorAgent*> connectorAgents =
        m_buildSpec.getConnectorAgents();

    for (std::size_t i = 0; i < connectorAgents.size(); i++)
    {
        tgBuildSpec::ConnectorAgent* const pConnectorAgent = connectorAgents[i];
        assert(pConnectorAgent != NULL);
        
        tgTagSearch tagSearch = tgTagSearch(pConnectorAgent->tagSearch);

        // Remove our tags so that subcomponents 'inherit' them (because of the
        // way tags work, removing a tag from the search is the same as adding 
        // the tag to children to be searched)
        tagSearch.remove(getTags());
        
        tgConnectorInfo* const pConnectorInfo = pConnectorAgent->infoFactory;
        assert(pConnectorInfo != NULL);

        // Note: we don't have to do nodes here since connectors are always based on pairs.

        // Pairs
        std::vector<tgConnectorInfo*> pairConnectors =
        pConnectorInfo->createConnectorInfos(m_structure.getPairs(), tagSearch);
        m_connectors.insert(m_connectors.end(), pairConnectors.begin(),
                pairConnectors.end());

    }
    
    // Children
    for (std::size_t i = 0; i < m_children.size(); i++)
    {
        tgStructureInfo * const pStructureInfo = m_children[i];
        assert(pStructureInfo != NULL);
        pStructureInfo->initConnectorInfo();
    }
}

void tgStructureInfo::chooseConnectorRigids()
{
    chooseConnectorRigids(getAllRigids());
}

void tgStructureInfo::chooseConnectorRigids(std::vector<tgRigidInfo*> allRigids)
{
    for (std::size_t i = 0; i < m_connectors.size(); i++)
    {
        tgConnectorInfo * const pConnectorInfo = m_connectors[i];
    assert(pConnectorInfo != NULL);
        pConnectorInfo->chooseRigids(allRigids);
    }    

    // Children
    for (std::size_t i = 0; i < m_children.size(); i++)
    {
        tgStructureInfo * const pStructureInfo = m_children[i];
    assert(pStructureInfo != NULL);
        pStructureInfo->chooseConnectorRigids(allRigids);
    }
}

void tgStructureInfo::initRigidBodies(tgWorld& world) 
{
    // Rigids
    for (std::size_t i = 0; i < m_rigids.size(); i++)
    {
        tgRigidInfo * const pRigidInfo = m_rigids[i];
    assert(pRigidInfo != NULL);
        pRigidInfo->initRigidBody(world);
    }
    
    // Children
    for (std::size_t i = 0; i < m_children.size(); i++)
    {
        tgStructureInfo * const pStructureInfo = m_children[i];
    assert(pStructureInfo != NULL);
        pStructureInfo->initRigidBodies(world);
    }
}

void tgStructureInfo::initConnectors(tgWorld& world) 
{
    // Connectors
    for (std::size_t i = 0; i < m_connectors.size(); i++)
    {
        tgConnectorInfo * const pConnectorInfo = m_connectors[i];
    assert(pConnectorInfo != NULL);
        pConnectorInfo->initConnector(world);
    }
    
    // Children
    for (std::size_t i = 0; i < m_children.size(); i++)
    {
        tgStructureInfo * const pStructureInfo = m_children[i];
    assert(pStructureInfo != NULL);
        pStructureInfo->initConnectors(world);
    } 
}

void tgStructureInfo::buildInto(tgModel& model, tgWorld& world) 
{
    // These take care of things on a global level
    initRigidInfo();
    autoCompoundRigids();    
    initConnectorInfo();
    chooseConnectorRigids();
    initRigidBodies(world);
    // Note: Muscle2Ps won't show up yet -- 
    // they need to be part of a model to have rendering...
    initConnectors(world);
    // Now build into the model
    buildIntoHelper(model, world, *this);    
}

void tgStructureInfo::buildIntoHelper(tgModel& model, tgWorld& world,
                      tgStructureInfo& structureInfo)
{
    
    const std::vector<tgRigidInfo*> rigids = structureInfo.getRigids();
    for (std::size_t i = 0; i < rigids.size(); i++)
    {
        tgRigidInfo * const pRigidInfo = rigids[i];
    assert(pRigidInfo != NULL);
        tgModel* const pModel = pRigidInfo->createModel(world);
        if (pModel != NULL)
    {
        pModel->setTags(pRigidInfo->getTags());
            model.addChild(pModel);
        }
    }
    
    const std::vector<tgConnectorInfo*> connectors = structureInfo.getConnectors();
    for (std::size_t i = 0; i < connectors.size(); i++)
    {
        tgConnectorInfo * const pConnectorInfo = connectors[i];
    assert(pConnectorInfo != NULL);
        tgModel* const pModel = pConnectorInfo->createModel(world);
        if (pModel != NULL)
    {      
            pModel->setTags(pConnectorInfo->getTags());      
            model.addChild(pModel);
        }
    }
    
    const std::vector<tgStructureInfo*> children = structureInfo.getChildren();
    for (std::size_t i = 0; i < children.size(); i++)
    {
        tgStructureInfo * const pStructureInfo = children[i];
    assert(pStructureInfo != NULL);
        tgModel* const pModel = new tgModel();
    assert(pModel != NULL);
        buildIntoHelper(*pModel, world, *pStructureInfo);
        model.addChild(pModel);
    }
    
    model.setTags(structureInfo.getTags());
}

void tgStructureInfo::addChild(tgStructureInfo* pChild)
{
    if (pChild == NULL)
    {
        throw std::invalid_argument("Child is NULL");
    }
    else
    {
        m_children.push_back(pChild);
    }
}
    
std::string tgStructureInfo::toString(const std::string& prefix) const 
{
        std::string p = "  ";
        std::ostringstream os;
        os << prefix << "tgStructureInfo(" << std::endl;

        os << prefix << p << "Rigids:" << std::endl;
        for (std::size_t i = 0; i < m_rigids.size(); i++) {
            os << prefix << p << p << *(m_rigids[i]) << std::endl;
        }

        os << prefix << p << "Connectors:" << std::endl;
        for (std::size_t i = 0; i < m_connectors.size(); i++) {
            os << prefix << p << p << *(m_connectors[i]) << std::endl;
        }

        os << prefix << p << "Children:" << std::endl;
        for (std::size_t i = 0; i < m_children.size(); i++) {
            os << m_children[i]->toString(prefix + p + p) << std::endl; 
        }

        os << prefix << p << "Tags: [" << getTags() << "]" << std::endl;

        os << prefix << ")" << std::endl;
        return os.str();
}

std::ostream&
operator<<(std::ostream& os, const tgStructureInfo& obj)
{
    os << obj.toString() << std::endl;
    return os;
}




