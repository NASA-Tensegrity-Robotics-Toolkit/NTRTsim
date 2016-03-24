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
 * $Id$
 */

#ifndef TG_YAML_STRING_HELPER_H
#define TG_YAML_STRING_HELPER_H

#include <string>
#include <ostream>
#include <sstream>

// Forward Declarations (needed due to recursion)
 std::string asYamlItems(const std::vector<tgStructure*> structures, int indentLevel=0);

/**
 * Represent tags as a yaml list
 */
inline std::string asYamlList(const tgTags& tags)
{
    std::stringstream os;
    os << "[";
    for(size_t i = 0; i < tags.size(); i++) {
        os << '"' << tags[i] << '"';
        if(i != tags.size() - 1) 
            os << ", ";
    }    
    os << "]";
    return os.str();
};

inline std::string asYamlItem(const tgNode& node, int indentLevel=0)
{
    std::stringstream os;
    std::string indent = std::string(2 * (indentLevel), ' ');
    os << indent << "- tags: " << asYamlList(node.getTags()) << std::endl;
    os << indent << "  xyz: [" << node.x() << ", " << node.y() << ", " << node.z() << "]" << std::endl;
    return os.str();
}

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
}

inline std::string asYamlItem(const tgPair& pair, int indentLevel=0)
{
    std::stringstream os;
    std::string indent = std::string(2 * (indentLevel), ' ');
    os << indent << "- tags: " << asYamlList(pair.getTags()) << std::endl;
    os << indent << "  pair: ["
        << "[" << pair.getFrom().x() << ", " << pair.getFrom().y() << ", " << pair.getFrom().z() << "]" << ", "
        << "[" << pair.getTo().x() << ", " << pair.getTo().y() << ", " << pair.getTo().z() << "]"
    << "]" << std::endl;
    return os.str();    
}

inline std::string asYamlItems(const tgPairs& pairs, int indentLevel=0)
{
    std::stringstream os;
    std::string indent = std::string(2 * (indentLevel), ' ');

    if (pairs.size() == 0) {
        os << indent << "pairs: []" << std::endl;
        return os.str();
    }

    os << indent << "pairs:" << std::endl;
    for(size_t i = 0; i < pairs.size(); i++)
    {
        os << asYamlItem(pairs[i], indentLevel+1);
    }
    return os.str();
}

/**
 * Represent a structure as an element
 */
inline std::string asYamlElement(const tgStructure& structure, int indentLevel=0)
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


/** 
 * Represent a structure as a list item (prepended by a dash)
 */
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

/**
 * Represent a vector of tgStructures as items
 */
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

// ...


#endif
