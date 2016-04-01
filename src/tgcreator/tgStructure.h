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
#include <queue>

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

    tgStructure(const tgStructure& orig);

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

    /*
     * Removes the pair that's passed in as a parameter from the structure
     * (added to accommodate structures encoded in YAML)
     * @param[in] pair a reference to the pair to remove
     */
    void removePair(const tgPair& pair);

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

    /*
     * Scales structure by a scale factor
     * @param[in] scaleFactor the scale factor by which to scale the structure
     */
    void scale(double scaleFactor);

    /*
     * Scales structure relative to a reference point
     * @param[in] referencePoint a btVector3 reference point to scale the structure from/to
     * @param[in] scaleFactor the scale factor by which to scale the structure
     */
    void scale(const btVector3& referencePoint, double scaleFactor);

    /**
     * Add a child structure. Note that this will be copied rather than
     * being a reference or a pointer.
     */
    void addChild(tgStructure* child);    

    void addChild(const tgStructure& child);

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
     * Looks through nodes that we own and those that belong to child nodes
     * (using BFS) and returns the first node with a matching name.
     * Throws an error if a node is not a found with a matching name.
     * (added to accommodate structures encoded in YAML)
     * @param[in] name the name of the node to find and return
     * @return a reference to the node that was found
     */
    tgNode& findNode(const std::string& name);

    /**
     * Returns the mean position of the nodes in the structure (including children)
     * (added to accommodate structures encoded in YAML)
     * @return a btVector3 that represents the centroid of the structure
     */
    btVector3 getCentroid() const;

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
     * Looks through pairs that we own and those that belong to child nodes
     * (using BFS) and returns the first pair with matching endpoint coordinates.
     * Throws an error if a pair is not a found.
     * (added to accommodate structures encoded in YAML)
     * @param[in] from the vector on one end of the pair to find and return
     * @param[in] to the vector on the other end of the pair to find and return
     * @return a reference to the pair that was found
     */
    tgPair& findPair(const btVector3& from, const btVector3& to);
	
    /**
     * Return our child structures
     */
    const std::vector<tgStructure*>& getChildren() const
    {
        return m_children;
    }

    /**
     * Looks through children we own and those that belong to our children (using BFS)
     * and returns the first child with a matching name.
     * Throws an error if a child is not a found with a matching name.
     * (added to accommodate structures encoded in YAML)
     * @param[in] name the name of the structure to find and return
     * @return a reference to the structure that was found
     */
    tgStructure& findChild(const std::string& name);

private:

    tgNodes m_nodes;

    tgPairs m_pairs;

    // we own these
    std::vector<tgStructure*> m_children;
    
};

/**
 * Overload operator<<() to handle a tgStructure
 * @param[in,out] os an ostream
 * @param[in] tgStructure, its nodes, pairs, and children.
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 */
 /*
inline std::ostream&
operator<<(std::ostream& os, const tgStructure& structure) 
{
  os << "tgStructure with the following tgNodes and tgPairs: " << std::endl;
  os << structure.getNodes() << std::endl;
  os << structure.getPairs() << std::endl;
  os << "This tgStructure has the following children (also tgStructures): "
     << std::endl;

  // add each child structure's string output to the output stream
  std::vector<tgStructure*> children = structure.getChildren();
  for(std::size_t i= 0; i < children.size(); i++)
  {
    os << "Child " << i << ": " << std::endl;
    os << *(children[i]) << std::endl;
  }

  // If this tgStructure has no children, say so.
  if( children.size() == 0)
  {
    os << "(no children)" << std::endl;
  }

  return os;
}
*/



/**
 * Represent a structure as a YAML element
 * Note: this function has no dependencies on external libraries
 */
std::string asYamlElement(const tgStructure& structure, int indentLevel=0);

/** 
 * Represent a structure as a YAML list item (prepended by '-', multi-line)
 * Note: this function has no dependencies on external libraries
 */
std::string asYamlItem(const tgStructure& structure, int indentLevel=0);

/**
 * Represent a vector of tgStructure* as YAML items
 * Note: this function has no dependencies on external libraries
 */
std::string asYamlItems(const std::vector<tgStructure*> structures, int indentLevel=0);

/**
 * Overload operator<<() to handle a tgStructure
 * @param[in,out] os an ostream
 * @param[in] tgStructure, its nodes, pairs, and children.
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 */
inline std::ostream&
operator<<(std::ostream& os, const tgStructure& structure) 
{
    //os << asYamlElement(os, structure) << std::endl;
    os << asYamlElement(structure) << std::endl;
    return os;
};

/*
std::string asYamlElement(const tgStructure& structure, int indentLevel=0)
{
    std::stringstream os;
    std::string indent = std::string(2 * (indentLevel), ' ');
    os << indent << "structure:" << std::endl;
    os << indent << "  tags: " << asYamlList(structure.getTags()) << std::endl;
    os << asYamlItems(structure.getNodes(), indentLevel + 1);
    os << asYamlItems(structure.getPairs(), indentLevel + 1);
    os << asYamlItems(structure.getChildren(), indentLevel + 1);
    return os.str();
};

inline std::string asYamlItem(const tgStructure& structure, int indentLevel=0)
{
    std::stringstream os;
    std::string indent = std::string(2 * (indentLevel), ' ');
    os << indent << "- tags: " << asYamlList(structure.getTags()) << std::endl;
    os << asYamlItems(structure.getNodes(), indentLevel + 1);
    os << asYamlItems(structure.getPairs(), indentLevel + 1);
    os << asYamlItems(structure.getChildren(), indentLevel + 1);
    return os.str();
};

inline std::string asYamlItems(const std::vector<tgStructure*> structures, int indentLevel)
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
};

*/
#endif
