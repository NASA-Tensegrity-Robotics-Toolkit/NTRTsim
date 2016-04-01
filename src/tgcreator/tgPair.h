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
 * @file tgPair.h
 * @brief Definition of class tgPair
 * @author Ryan Adams
 * @date March 2014
 * $Id$
 */

#ifndef TG_PAIR_H
#define TG_PAIR_H

#include "core/tgTaggable.h"

#include "LinearMath/btVector3.h"

#include <string>
#include <iostream>

class btQuaternion;

/**
 * Change the API to std::pair<btVector3*, btVector3*>.
 * This renames "first" to "from" and "second" to "to". The client uses
 * non-const references.
 * @note: this maintains references to btVector3& since we can have a 
 * std::pair of references in c++03
 * @see http://stackoverflow.com/questions/3769781/stdpair-of-references
 * @todo Replace with "typedef tgPair std::pair<btVector3*, btVector3*>;"
 */
class tgPair : public tgTaggable
{
public:

    tgPair();

    /**
     * Create a pair from two btVector3 objects.
     * @param[in] from a btVector3
     * @param[in] to a btVector3
     * @todo Is it OK for from == to, either the same object or the same value?
     */
    tgPair(btVector3 from, btVector3 to);

    tgPair(btVector3 from, btVector3 to, std::string tags);
        
   /**
    * Return the from (first) member of the pair.
    * @return the from (first) member of the pair
    * @warning This will fail if the default constructor is used and setFrom()
    * is not called first.
    */
    btVector3& getFrom();

    const btVector3& getFrom() const;

    /**
     * Set the from (first) member of the pair.
     * @param[in] from the to (first) member of the pair
     */
    void setFrom(btVector3 from);
    
   /**
    * Return the to (second) member of the pair.
    * @return the to (second) member of the pair
    * @warning This will fail if the default constructor is used and setTo()
    * is not called first.
    */
    btVector3& getTo();

    const btVector3& getTo() const;

    /**
     * Set the to (second) member of the pair.
     * @param[in] to the to (second) member of the pair
     */
    void setTo(btVector3 to);
    
    void addRotation(const btVector3& fixedPoint,
                     const btVector3& axis,
                     double angle);
                     
    void addRotation(const btVector3& fixedPoint,
                     const btVector3& fromOrientation,
                     const btVector3& toOrientation);

    void addRotation(const btVector3& fixedPoint,
                     const btQuaternion& rotation);
    
    // Note: without this, tgTaggables will throw a 'Most vexing parse' error
    // @see http://en.wikipedia.org/wiki/Most_vexing_parse
    // @todo: Should we also compare the tags here? I think not...
    inline bool operator==(const tgPair& other) const
    {
        return (this->getFrom() == other.getFrom() && this->getTo() == other.getTo());
    }
        
    void move(const btVector3& offset);

    /*
     * Scales pair relative to a reference point
     * @param[in] referencePoint a btVector3 reference point to scale the pair from/to
     * @param[in] scaleFactor the scale factor by which to scale the pair
     */
    void scale(const btVector3& referencePoint, double scaleFactor);
    
protected:

    /** The underlying representation of the pair of btVector3 objects. */
    std::pair<btVector3, btVector3> m_pair;
};

/**
 * Overload operator<<() to handle a tgPair
 * @param[in,out] os an ostream
 * @param[in] pair a tgPair
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 */
inline std::ostream&
operator<<(std::ostream& os, const tgPair& pair) 
{
        os << "tgPair(" << pair.getFrom() << ", " << pair.getTo() 
            << ", {" << pair.getTagStr(", ") << "})";
    return os;
}


/**
 * Represent a pair as a YAML item (prepended by '-', multi-line)
 * Note: this function has no dependencies on external libraries
 */
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
};


#endif
