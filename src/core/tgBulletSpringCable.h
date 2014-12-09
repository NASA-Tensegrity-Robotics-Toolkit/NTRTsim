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

#ifndef SRC_CORE_TG_BULLET_SPRING_CABLE_H_
#define SRC_CORE_TG_BULLET_SPRING_CABLE_H_

/**
 * @file tgBulletSpringCable.h
 * @brief Definitions of classes tgBulletSpringCable
 * $Id$
 */

// NTRT
#include "tgSpringCable.h"

// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library

#include <vector>

// Forward references
class btRigidBody;
class tgSpringCableAnchor;
class tgBulletSpringCableAnchor;

class tgBulletSpringCable : public tgSpringCable
{
public: 
    // Alternative constructor
    tgBulletSpringCable( const std::vector<tgBulletSpringCableAnchor*>& anchors,
                double coefK,
                double dampingCoefficient,
                double pretension = 0.0);
    
    virtual ~tgBulletSpringCable();

    // Called by tensegrity class update function for each muscle2p
    virtual void step(double dt);
   
    virtual const double getRestLength() const;
    
    virtual void setRestLength( const double newRestLength); 

    virtual const double getActualLength() const;

    virtual const double getTension() const;
    
    /**
     * @todo figure out how to cast and pass by reference
     * @todo should the anchors themselves be const?
     */
    virtual const std::vector<tgSpringCableAnchor*> getAnchors() const;
     
protected:

   // Wanted to do a set, but need random access iterator to sort
   // Needs to be stored here for consistent rendering
   std::vector<tgBulletSpringCableAnchor*> m_anchors;

protected:
    tgBulletSpringCableAnchor * const anchor1;

    tgBulletSpringCableAnchor * const anchor2;
    
private:

    virtual void calculateAndApplyForce(double dt);

 private: 
    bool invariant(void) const;
};

#endif  // SRC_CORE_TG_BULLET_SPRING_CABLE_H_
