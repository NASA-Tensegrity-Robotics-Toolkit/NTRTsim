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

#ifndef TG_NODE_H
#define TG_NODE_H

/**
 * @file tgNode.h
 * @brief Definition of class tgNode
 * @author Ryan Adams
 * @date March 2014
 * $Id$
 */

#include "tgUtil.h"
#include "core/tgTaggable.h"

//Bullet Physics
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"

// NOTE: Should tgNode hold a quaternion to specify rotation/orientation? 


/**
 * An extension of btVector3 that adds taggability,
 * rotation, and other editing features (to be added).
 * Tagging just a node can allow a tgSphere to be placed there.
 */
class tgNode : public btVector3, public tgTaggable
{
public:

    tgNode(const btVector3& v = btVector3(),
             const std::string& tags = "") :
        btVector3(v), tgTaggable(tags)  
    {}
        
    tgNode(double x, double y, double z, 
             const std::string& tags = "") : 
        btVector3(x,y,z), tgTaggable(tags)
    {}

    /**
     * Rotate this vector around the specified fixed point
     */
    void addRotation(const btVector3& fixedPoint, 
                     const btVector3& fromOrientation,
                     const btVector3& toOrientation)
    {
        // Note: there's likely a more efficient way to do this...
        addRotation(fixedPoint, 
                    tgUtil::getQuaternionBetween(fromOrientation, 
                                                 toOrientation));
    }

    /**
     * Rotate this vector around the specified fixed point
     */
    void addRotation(const btVector3& fixedPoint, 
                     const btVector3& axis,
                     double angle)
    {
        tgUtil::addRotation(*this, fixedPoint, axis, angle);       
    }

    
    /**
     * Rotate this vector around the specified fixed point
     */
    void addRotation(const btVector3& fixedPoint, 
                     const btQuaternion& rotation)
    {
        addRotation(fixedPoint, 
                    rotation.getAxis(), 
                    rotation.getAngle());
    }    

    // Note: Required for extending tgTaggable
    bool operator==(const tgNode& other) const
    {
      return (this->x() == other.x()) &&
             (this->y() == other.y()) &&
             (this->z() == other.z());
    }

};



/**
 * Overload operator<<() to handle a tgNode
 * @param[in,out] os an ostream
 * @param[in] pair a tgNode
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 */
inline std::ostream&
operator<<(std::ostream& os, const tgNode& node) 
{
    os << "tgNode(" << node.x() << ", " << node.y() << ", " << node.z() << ", {" << node.getTagStr(", ") << "})";
    return os;
};


/**
 * Represent a node as a YAML item (prepended by '-', multi-line)
 * Note: this function has no dependencies on external libraries
 */
inline std::string asYamlItem(const tgNode& node, int indentLevel=0)
{
    std::stringstream os;
    std::string indent = std::string(2 * (indentLevel), ' ');
    os << indent << "- tags: " << asYamlList(node.getTags()) << std::endl;
    os << indent << "  xyz: [" << node.x() << ", " << node.y() << ", " << node.z() << "]" << std::endl;
    return os.str();
};


#endif
