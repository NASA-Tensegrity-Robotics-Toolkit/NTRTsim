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

#ifndef NTRT_MUSCLE2P_H_
#define NTRT_MUSCLE2P_H_

/**
 * @file Muscle2P.h
 * @brief Definitions of classes Muscle2P
 * $Id$
 */

// NTRT
#include "tgSpringCable.h"

// The Bullet Physics library
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
// The C++ Standard Library

#include <vector>

// Forward references
class btRigidBody;
class tgSpringCableAnchor;
class tgBulletSpringCableAnchor;

class Muscle2P : public tgSpringCable
{
public:	
	// Alternative constructor
	Muscle2P( const std::vector<tgBulletSpringCableAnchor*>& anchors,
				double coefK,
				double dampingCoefficient,
				double pretension = 0.0);
	
    virtual ~Muscle2P();

    /// @todo remove this
    virtual void calculateAndApplyForce(double dt);
    // Called by tensegrity class update function for each muscle2p
    virtual void step(double dt);
   
    virtual const double getRestLength() const;
    
    virtual void setRestLength( const double newRestLength); 

    virtual const double getActualLength() const;

    virtual const double getTension() const;
    
    /**
     * @todo figure out how to cast and pass by reference
     */
    virtual const std::vector<tgSpringCableAnchor*> getAnchors() const;
    
/// @todo make these protected, force other functions to use getAnchors()   
    tgBulletSpringCableAnchor * const anchor1;

    tgBulletSpringCableAnchor * const anchor2;
    
protected:

   // Wanted to do a set, but need random access iterator to sort
   // Needs to be stored here for consistent rendering
   std::vector<tgBulletSpringCableAnchor*> m_anchors;


 private: 
    bool invariant(void) const;
};

#endif  // NTRT_MUSCLE2P_H_
