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
 * @file tgPair.cpp
 * @brief Definition of class tgPair
 * @author Ryan Adams
 * @date March 2014
 * $Id$
 */

#include "tgPair.h"

#include "tgUtil.h"


tgPair::tgPair() : tgTaggable() {}

/**
 * Create a pair from two btVector3 objects.
 * @param[in] from a btVector3
 * @param[in] to a btVector3
 * @todo Is it OK for from == to, either the same object or the same value?
 */
tgPair::tgPair(btVector3 from, btVector3 to) : m_pair(from, to), tgTaggable() 
{}

tgPair::tgPair(btVector3 from, btVector3 to, std::string tags) : m_pair(from, to), tgTaggable(tags) 
{}
    
/**
* Return the from (first) member of the pair.
* @return the from (first) member of the pair
* @warning This will fail if the default constructor is used and setFrom()
* is not called first.
*/
btVector3& tgPair::getFrom()
{
    assert(m_pair.first != NULL);
    return m_pair.first;
}

const btVector3& tgPair::getFrom() const
{
    assert(m_pair.first != NULL);
    return m_pair.first;
}

/**
 * Set the from (first) member of the pair.
 * @param[in] from the to (first) member of the pair
 */
void tgPair::setFrom(btVector3 from) { m_pair.first = from; }

/**
* Return the to (second) member of the pair.
* @return the to (second) member of the pair
* @warning This will fail if the default constructor is used and setTo()
* is not called first.
*/
btVector3& tgPair::getTo() 
{
    assert(m_pair.second != NULL);
    return m_pair.second;
}

const btVector3& tgPair::getTo() const
{
    assert(m_pair.second != NULL);
    return m_pair.second;
}

/**
 * Set the to (second) member of the pair.
 * @param[in] to the to (second) member of the pair
 */
void tgPair::setTo(btVector3 to) 
{ 
    m_pair.second = to; 
}
    
void tgPair::addRotation(const btVector3& fixedPoint,
                 const btVector3& axis,
                 double angle)
{
    tgUtil::addRotation(getFrom(), fixedPoint, axis, angle);
    tgUtil::addRotation(getTo(), fixedPoint, axis, angle);
}

void tgPair::addRotation(const btVector3& fixedPoint,
                 const btVector3& fromOrientation,
                 const btVector3& toOrientation)
{
    tgUtil::addRotation(getFrom(), fixedPoint, fromOrientation, toOrientation);
    tgUtil::addRotation(getTo(), fixedPoint, fromOrientation, toOrientation);
}

void tgPair::addRotation(const btVector3& fixedPoint,
                 const btQuaternion& rotation)
{
    tgUtil::addRotation(getFrom(), fixedPoint, rotation);
    tgUtil::addRotation(getTo(), fixedPoint, rotation);
}
    
void tgPair::move(const btVector3& offset)
{
    m_pair.first += offset;
    m_pair.second += offset;
}

void tgPair::scale(const btVector3& referencePoint, double scaleFactor) {
    m_pair.first.setX((m_pair.first.x() - referencePoint.x()) * scaleFactor + referencePoint.x());
    m_pair.first.setY((m_pair.first.y() - referencePoint.y()) * scaleFactor + referencePoint.y());
    m_pair.first.setZ((m_pair.first.z() - referencePoint.z()) * scaleFactor + referencePoint.z());

    m_pair.second.setX((m_pair.second.x() - referencePoint.x()) * scaleFactor + referencePoint.x());
    m_pair.second.setY((m_pair.second.y() - referencePoint.y()) * scaleFactor + referencePoint.y());
    m_pair.second.setZ((m_pair.second.z() - referencePoint.z()) * scaleFactor + referencePoint.z());
}

