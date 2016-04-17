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

#ifndef TG_NODES_H
#define TG_NODES_H

/**
 * @file tgNodes.h
 * @brief Definition of class tgNodes 
 * @author Ryan Adams
 * @date January 2014
 * $Id$
 */

#include "LinearMath/btVector3.h"
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <stdexcept>
#include <sstream>

#include "tgNode.h"
#include "core/tgTaggables.h"

class tgPair;
 
/**
 * A node is an attachment point. The client identifies nodes with an integer
 * index or selects them using tags (see tgTaggable). 
 * @TODO: add error checking
 * @todo: move operator[] out of tgTaggables into here
 */
class tgNodes : public tgTaggables<tgNode> {
public:
    
    /**
     * Create an empty set of nodes.
     */
    tgNodes() : tgTaggables()
    {
    }

    /**
     * Create a set of nodes given a vector of btVector3.
     * @param[in] nodes a vector of btVector3; the elements must be unique
     * @author Lee Brownston
     * @date Wed 26 Feb 2014
     */
    tgNodes(std::vector<btVector3>& nodes) : tgTaggables()
    {
        // All elements must be unique
        assertUniqueElements("All nodes must be unique.");

        // @todo: There has to be a better way to do this (maybe initializer lists with upcasting btVector3 => tgNode?) 
        for(std::size_t i = 0; i < nodes.size(); i++) {
            addElement(tgNode(nodes[i]));
        }
        
    }
     
    tgNodes(std::vector<tgNode>& nodes) : tgTaggables(nodes) {
        // All elements must be unique
        assertUniqueElements("All nodes must be unique.");
        
    }
        
    /** The destructor has nothing to do. */
    ~tgNodes() { }
    
    /**
     * Assign the given vector to the given key.
     * @param[in] node a btVector3
     * @param[idx] the integer by which the btVector3 is indexed
     */
    void setNode(int key, const btVector3& node)
    {
        setNode(key, tgNode(node));
    };

    void setNode(int key, const tgNode& node)
    {
        setElement(key, node);
    }
    
    std::vector<tgNode>& getNodes() 
    {
        return getElements();
    };

    const std::vector<tgNode>& getNodes() const
    {
        return getElements();
    };


    /**
     * Is the index within range.
     * @param[in] key an int
     * @retval true if key is within range
     * @retval false if key is not within range
     */
    bool nodeExists(int key) const
    {
        return keyExists(key);
    }

    /**
     * Add a node and return the created index.
     * @param[in] node a btVector3
     * @return the key under which btVector3 is stored
     * @todo If node is already a value in the map, return its key.
     */
    int addNode(const btVector3& node) {
        return addNode(tgNode(node));
    };

    int addNode(const btVector3& node, std::string tags) {
        return addNode(tgNode(node, tags));
    };
    
    int addNode(const tgNode& node) {
        return addElement(node);
    }

    /**
     * Add a node specified by its coordinates and return the created index.
     * @param[in] x the x coordinate of the btVector3
     * @param[in] y the y coordinate of the btVector3
     * @param[in] z the z coordinate of the btVector3
     * @return the key under which btVector3 is stored
     * @TODO: Do we need this really? It complicates things like named nodes...
     * @todo If node is already a value in the map, return its index.
     */
     // @todo: Maybe replace this with something like:
     //     myNodes += tgNode(1,2,3, "some tags");
     //     or just use myNodes.addNode(tgNode(1,2,3, "tags here"));
    int addNode(double x, double y, double z)
    {
        const btVector3 node(x, y, z);
        return addNode(node);
    }

    int addNode(double x, double y, double z, std::string tags)
    {
        const tgNode node(x, y, z, tags);
        return addNode(node);
    }

    /**
     * Create a tgPair by connecting two contained nodes
     */
    tgPair pair(int from, int to, std::string tags = "");

    /**
     * Add the given btVector3 to all btVector3 objects in elements.
     * @param[in] offset a btVector3 to add to all the btVector3 objects in m_nodes
     * 
     */
    void move(const btVector3& offset)
    {
        /// @todo use std::for_each()
        std::vector<tgNode>& nodes = getElements();
        for(std::size_t i = 0; i < nodes.size(); i++) {
            nodes[i] += offset;
        }
    }

    void moveNode(int idx, const btVector3 offset)
    {
        (*this)[idx] += offset;
    }
    
    void addRotation(const btVector3& fixedPoint,
                     const btVector3& axis,
                     double angle)
    {
        btQuaternion rotation(axis, angle);
        addRotation(fixedPoint, rotation);
    }

    void addRotation(const btVector3& fixedPoint,
                     const btVector3& fromOrientation,
                     const btVector3& toOrientation)
    {
        btQuaternion rotation = tgUtil::getQuaternionBetween(fromOrientation, 
                                                             toOrientation);
        addRotation(fixedPoint, rotation);
    }

    void addRotation(const btVector3& fixedPoint,
                     const btQuaternion& rotation)
    {
        std::vector<tgNode>& nodes = getNodes();
        for(std::size_t i = 0; i < nodes.size(); i++) {
            nodes[i].addRotation(fixedPoint, rotation);
        }
    }

    /*
     * Scales nodes relative to a reference point
     * @param[in] referencePoint a btVector3 reference point to scale the nodes from/to
     * @param[in] scaleFactor the scale factor by which to scale the nodes
     */
    void scale(const btVector3& referencePoint, double scaleFactor) {
        std::vector<tgNode>& nodes = getNodes();
        for(int i = 0; i < nodes.size(); i++) {
            nodes[i].setX((nodes[i].x() - referencePoint.x()) * scaleFactor + referencePoint.x());
            nodes[i].setY((nodes[i].y() - referencePoint.y()) * scaleFactor + referencePoint.y());
            nodes[i].setZ((nodes[i].z() - referencePoint.z()) * scaleFactor + referencePoint.z());
        }
    }
    
protected:
    
    // A map of m_nodes keys to names. Note that not all m_nodes will have names.
    std::map<int, std::string> m_names;  // @todo: remove this...
    
    void assertNodeExists(int key) const
    {
        if(!keyExists(key)) {
            std::stringstream ss; 
            ss << key;
            throw std::out_of_range("Node at index " + ss.str() + " does not exist");
        }        
    }
    
    void assertUniqueNodes() const
    {
        assertUniqueElements("Nodes muse be unique.");
    }
    
};


/**
 * Overload operator<<() to handle tgNodes
 * @param[in,out] os an ostream
 * @param[in] pair a tgNodes
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 */
inline std::ostream&
operator<<(std::ostream& os, const tgNodes& n)
{
    os << "tgNodes(" << std::endl;
    const std::vector<tgNode>& nodes = n.getNodes();
    for(std::size_t i = 0; i < nodes.size(); i++) {
        os << "  " << nodes[i] << std::endl;
    }
    os << ")";

    return os;
};


/**
 * Represent nodes as a YAML list (prepended by '-', multi-line)
 * Note: this function has no dependencies on external libraries
 */
inline std::string asYamlItems(const tgNodes& nodes, int indentLevel=0)
{
    std::stringstream os;
    std::string indent = std::string(2 * (indentLevel), ' ');

    if (nodes.size() == 0) {
        os << indent << "nodes: []" << std::endl;
        return os.str();
    }

    os << indent << "nodes:" << std::endl;
    for(size_t i = 0; i < nodes.size(); i++)
    {
        os << asYamlItem(nodes[i], indentLevel+1);
    }
    return os.str();
};



#endif
