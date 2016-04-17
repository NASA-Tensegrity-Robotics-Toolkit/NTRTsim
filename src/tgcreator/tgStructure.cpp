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
 * @file tgStructure.cpp
 * @brief Implementation of class tgStructure
 * @author Ryan Adams
 * @date March 2014
 * $Id$
 */

// This module
#include "tgStructure.h"
// This library
#include "tgNode.h"
#include "tgPair.h"
// The Bullet Physics library
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>
 
tgStructure::tgStructure() : tgTaggable() 
{
}


/**
 * Copy constructor
 */
tgStructure::tgStructure(const tgStructure& orig) : tgTaggable(orig.getTags()), 
        m_children(orig.m_children.size()), m_nodes(orig.m_nodes), m_pairs(orig.m_pairs)
{
    
    // Copy children
    for (std::size_t i = 0; i < orig.m_children.size(); ++i) {
        m_children[i] = new tgStructure(*orig.m_children[i]);
    }
}

tgStructure::tgStructure(const tgTags& tags) : tgTaggable(tags)
{
}

tgStructure::tgStructure(const std::string& space_separated_tags) : tgTaggable(space_separated_tags)
{
}

tgStructure::~tgStructure()
{
    for (std::size_t i = 0; i < m_children.size(); ++i)
    {
        delete m_children[i];
    }
}

void tgStructure::addNode(double x, double y, double z, std::string tags)
{
    m_nodes.addNode(x, y, z, tags);
}

void tgStructure::addNode(tgNode& newNode)
{
    m_nodes.addNode(newNode);
}

void tgStructure::addPair(int fromNodeIdx, int toNodeIdx, std::string tags)
{
    addPair(m_nodes[fromNodeIdx], m_nodes[toNodeIdx], tags);
}

void tgStructure::addPair(const btVector3& from, const btVector3& to, std::string tags)
{
    // @todo: do we need to pass in tags here? might be able to save some proc time if not...
    tgPair p = tgPair(from, to);
    if (!m_pairs.contains(p))
    {
        m_pairs.addPair(tgPair(from, to, tags));
    }
    else
    {
        std::ostringstream os;
        os << "A pair matching " << p << " already exists in this structure.";
        throw tgException(os.str());
    }
}

void tgStructure::removePair(const tgPair& pair) {
    m_pairs.removePair(pair);
    for (unsigned int i = 0; i < m_children.size(); i++) {
        m_children[i]->removePair(pair);
    }
}

void tgStructure::move(const btVector3& offset)
{
    m_nodes.move(offset);
    m_pairs.move(offset);
    for (size_t i = 0; i < m_children.size(); ++i)
    {
        tgStructure * const pStructure = m_children[i];
    assert(pStructure != NULL);
        pStructure->move(offset);
    }
}

void tgStructure::addRotation(const btVector3& fixedPoint,
                 const btVector3& axis,
                 double angle)
{
    const btQuaternion rotation(axis, angle);
    addRotation(fixedPoint, rotation);
}

void tgStructure::addRotation(const btVector3& fixedPoint,
                 const btVector3& fromOrientation,
                 const btVector3& toOrientation)
{
    addRotation(fixedPoint, 
                tgUtil::getQuaternionBetween(fromOrientation, 
                                             toOrientation));
}

void tgStructure::addRotation(const btVector3& fixedPoint,
                 const btQuaternion& rotation)
{
    m_nodes.addRotation(fixedPoint, rotation);
    m_pairs.addRotation(fixedPoint, rotation);

    for (std::size_t i = 0; i < m_children.size(); ++i)
    {
        tgStructure * const pStructure = m_children[i];
    assert(pStructure != NULL);
        pStructure->addRotation(fixedPoint, rotation);
    }
}

void tgStructure::scale(double scaleFactor) {
    btVector3 structureCentroid = getCentroid();
    scale(structureCentroid, scaleFactor);
}

void tgStructure::scale(const btVector3& referencePoint, double scaleFactor) {
    m_nodes.scale(referencePoint, scaleFactor);
    m_pairs.scale(referencePoint, scaleFactor);

    for (int i = 0; i < m_children.size(); i++) {
        tgStructure* const childStructure = m_children[i];
        assert(childStructure != NULL);
        childStructure->scale(referencePoint, scaleFactor);
    }
}

void tgStructure::addChild(tgStructure* pChild)
{
    /// @todo: check to make sure we don't already have one of these structures
    /// (what does that mean?)
    /// @note: We only want to check that pairs are the same at build time, since one
    /// structure may build the pairs, while another may not depending on its tags.
    if (pChild != NULL)
    {
        m_children.push_back(pChild);
    }
}

void tgStructure::addChild(const tgStructure& child)
{
    m_children.push_back(new tgStructure(child));
    
}

btVector3 tgStructure::getCentroid() const {
    btVector3 centroid = btVector3(0, 0, 0);
    int numNodes = 0;
    std::queue< const tgStructure*> q;

    q.push(this);

    while (!q.empty()) {
        const tgStructure* structure = q.front();
        q.pop();
        for (int i = 0; i < structure->m_nodes.size(); i++) {
            centroid += structure->m_nodes[i];
            numNodes++;
        }
        for (int i = 0; i < structure->m_children.size(); i++) {
            q.push(structure->m_children[i]);
        }
    }
    return centroid/numNodes;
}

tgNode& tgStructure::findNode(const std::string& tags) {
    std::queue<tgStructure*> q;

    q.push(this);

    while (!q.empty()) {
        tgStructure* structure = q.front();
        q.pop();
        for (int i = 0; i < structure->m_nodes.size(); i++) {
            if (structure->m_nodes[i].hasAllTags(tags)) {
                return structure->m_nodes[i];
            }
        }
        for (int i = 0; i < structure->m_children.size(); i++) {
            q.push(structure->m_children[i]);
        }
    }
    throw std::invalid_argument("Node not found: " + tags);
}

tgPair& tgStructure::findPair(const btVector3& from, const btVector3& to) {
    std::queue<tgStructure*> q;

    q.push(this);

    while (!q.empty()) {
        tgStructure* structure = q.front();
        q.pop();
        for (int i = 0; i < structure->m_pairs.size(); i++) {
            if ((structure->m_pairs[i].getFrom() == from && structure->m_pairs[i].getTo() == to) ||
                (structure->m_pairs[i].getFrom() == to && structure->m_pairs[i].getTo() == from)) {
                return structure->m_pairs[i];
            }
        }
        for (int i = 0; i < structure->m_children.size(); i++) {
            q.push(structure->m_children[i]);
        }
    }
    std::ostringstream pairString;
    pairString << from << ", " << to;
    throw std::invalid_argument("Pair not found: " + pairString.str());
}

tgStructure& tgStructure::findChild(const std::string& tags) {
    std::queue<tgStructure*> q;

    for (int i = 0; i < m_children.size(); i++) {
        q.push(m_children[i]);
    }

    while (!q.empty()) {
        tgStructure* structure = q.front();
        q.pop();
        if (structure->hasAllTags(tags)) {
            return *structure;
        }
        for (int i = 0; i < structure->m_children.size(); i++) {
            q.push(structure->m_children[i]);
        }
    }
    throw std::invalid_argument("Child structure not found: " + tags);
}

/* Standalone functions */
std::string asYamlElement(const tgStructure& structure, int indentLevel)
{
    std::stringstream os;
    std::string indent = std::string(2 * (indentLevel), ' ');
    os << indent << "structure:" << std::endl;
    os << indent << "  tags: " << asYamlList(structure.getTags()) << std::endl;
    os << asYamlItems(structure.getNodes(), indentLevel + 1);
    os << asYamlItems(structure.getPairs(), indentLevel + 1);
    os << asYamlItems(structure.getChildren(), indentLevel + 1);
    return os.str();
}

std::string asYamlItem(const tgStructure& structure, int indentLevel)
{
    std::stringstream os;
    std::string indent = std::string(2 * (indentLevel), ' ');
    os << indent << "- tags: " << asYamlList(structure.getTags()) << std::endl;
    os << asYamlItems(structure.getNodes(), indentLevel + 1);
    os << asYamlItems(structure.getPairs(), indentLevel + 1);
    os << asYamlItems(structure.getChildren(), indentLevel + 1);
    return os.str();
}

std::string asYamlItems(const std::vector<tgStructure*> structures, int indentLevel)
{
    std::stringstream os;
    std::string indent = std::string(2 * (indentLevel), ' ');
    if (structures.size() == 0) {
        os << indent << "structures: []" << std::endl;
        return os.str();
    }

    os << indent << "structures:" << std::endl;
    for(size_t i = 0; i < structures.size(); i++)
    {
        os << asYamlItem(*structures[i], indentLevel+1);
    }
    return os.str();
}
