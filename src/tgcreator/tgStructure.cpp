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
 
tgStructure::tgStructure() : tgComponent(), m_nodes(), m_pairs()
{
}

tgStructure::tgStructure(const tgTags& tags) : tgComponent(tags), m_nodes(), m_pairs()
{
}

tgStructure::tgStructure(const std::string& space_separated_tags) : tgComponent(space_separated_tags), m_nodes(), m_pairs()
{
}

tgStructure::~tgStructure()
{
    for (std::size_t i = 0; i < m_children.size(); ++i)
    {
        delete m_children[i];
    }
}

void tgStructure::addNode(double x, double y, double z, const std::string& tags, const std::string& name)
{
    m_nodes.addNode(x, y, z, tags, name);
}

/*
void tgStructure::addNode(tgNode& newNode)
{
    m_nodes.addNode(newNode);
}
*/
void tgStructure::addNode(const tgNode& newNode)
{
    //std::cout << "tgStructure::addNode(const tgNode& newNode)" << std::endl;
    m_nodes.addNode(newNode);
}

void tgStructure::addPair(int fromNodeIdx, int toNodeIdx, const std::string& tags, const std::string& name)
{
    addPair(m_nodes[fromNodeIdx], m_nodes[toNodeIdx], tags, name);
}

void tgStructure::addPair(const btVector3& from, const btVector3& to, const std::string& tags, const std::string& name)
{
    // @todo: do we need to pass in tags here? might be able to save some proc time if not...
    tgPair p = tgPair(from, to);
    if (!m_pairs.contains(p))
    {
        m_pairs.addPair(tgPair(from, to, tags, name));
    }
    else
    {
        std::ostringstream os;
        os << "A pair matching " << p << " already exists in this structure.";
        throw tgException(os.str());
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

void tgStructure::addChild(tgStructure* pChild, const std::string& name)
{
    /// @todo: check to make sure we don't already have one of these structures
    /// (what does that mean?)
    /// @note: We only want to check that pairs are the same at build time, since one
    /// structure may build the pairs, while another may not depending on its tags.
    // @todo: handle name -- since this is a pointer, can we set the name without causing problems? For that matter, can we move, rotate, etc? 
    if (pChild != NULL)
    {
        m_children.push_back(pChild);
    }
}
