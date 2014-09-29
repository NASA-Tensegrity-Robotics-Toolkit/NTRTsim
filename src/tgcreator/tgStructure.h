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

#ifndef TG_STRUCTURE_H
#define TG_STRUCTURE_H

/**
 * @file tgStructure.h
 * @brief Definition of class tgStructure
 * @author Ryan Adams
 * @date March 2014
 * $Id$
 */

// This library
#include "tgNodes.h"
#include "tgPairs.h"
// The NTRT Core Library
#include "core/tgTaggable.h"
// The C++ Standard Library
#include <string>
#include <vector>

// Forward declarations
class btQuaternion;
class btVector3;
class tgNode;
class tgTags;

/**
 * Representation of a geometric structure containing nodes (points), pairs of
 * nodes, and child structures. Note that nodes, pairs, and structures are all 
 * taggable and searchable by tags. Further tools in the chain are used to
 * create physical representations of the structures with rods, muscles, etc.
 * Note that tags can be anything you want -- you'll specify the tags that you 
 * want to use to build things like rods or muscles during the build phase.
 */
class tgStructure : public tgTaggable
{
public:
    
    tgStructure();

    tgStructure(const tgTags& tags);

    tgStructure(const std::string& space_separated_tags);

    virtual ~tgStructure();

    /**
     * Add a node using x, y, and z (just for convenience)
     */
    void addNode(double x, double y, double z, std::string tags = "");
    
    /**
     * Add a node using a node - since keeping track of nodes seems
     * more useful than pairs for string attachments
     */
    void addNode(tgNode& newNode);

    /**
     * Add a pair that connects two of our nodes together
     */
    void addPair(int fromNodeIdx, int toNodeIdx, std::string tags = "");
    
    /**
     * Add a pair that connects any two vector3s 
     */
    void addPair(const btVector3& from, const btVector3& to, std::string tags = "");

    void move(const btVector3& offset);
    
    /**
     * @todo add rotate functionality
     */
    void addRotation(const btVector3& fixedPoint,
             const btVector3& axis,
             double angle);

    void addRotation(const btVector3& fixedPoint,
             const btVector3& fromOrientation,
             const btVector3& toOrientation);

    void addRotation(const btVector3& fixedPoint,
             const btQuaternion& rotation);

    /**
     * Add a child structure. Note that this will be copied rather than
     * being a reference or a pointer.
     */
    void addChild(tgStructure* child);    

    /**
     * Get all of our nodes
     * Note: This only includes nodes owned by this structure. use 'findNodes'
     * to search child nodes as well.
     */
    const tgNodes& getNodes() const
    {
        return m_nodes;
    }

    /**
     * Get all of our pairs
     * Note: This only includes nodes owned by this structure. Use 'findPairs' 
     * to search child nodes as well. 
     */
    const tgPairs& getPairs() const
    {
        return m_pairs;
    }
	
    /**
     * Return our child structures
     */
    const std::vector<tgStructure*>& getChildren() const
    {
        return m_children;
    }

private:

    tgNodes m_nodes;

    tgPairs m_pairs;

    // we own these
    std::vector<tgStructure*> m_children;
    
};


#endif
