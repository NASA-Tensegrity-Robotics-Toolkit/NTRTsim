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
 * @brief Definitions of classes Muscle2P and MuscleAnchor.
 * @todo Split so only one class is defined per header file.
 * $Id$
 */

// The Bullet Physics library
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
// The C++ Standard Library

#include <string>
#include <vector>

// Forward references
class btRigidBody;
class muscleAnchor;

class Muscle2P
{
public:
    Muscle2P(btRigidBody * body1,
         btVector3 pos1,
         btRigidBody * body2,
         btVector3 pos2,
         double coefK,
         double dampingCoefficient);

    virtual ~Muscle2P();

    // Called by tensegrity class update function for each muscle2p
    virtual btVector3 calculateAndApplyForce(double dt);
    
    void setName(std::string a) { name = a; }
    
    const double getRestLength() const;
    
    void setRestLength( const double newRestLength); 

    virtual const btScalar getActualLength() const;

    const double getTension() const;
    
    const double getCoefK() const
    {
        return m_coefK;
    }
    
    const double getVelocity() const
    {
        return m_velocity;
    }
    
    const double getDamping() const
    {
        return m_damping;
    }

    const std::string& getName() const {
        return name;
    }
    
    muscleAnchor * const anchor1;

    muscleAnchor * const anchor2;

	const std::vector<muscleAnchor*>& getAnchors() const
    {
        return m_anchors;
    }

    std::string name;

    bool recordHistory;

protected:

   // Wanted to do a set, but need random access iterator to sort
   // Needs to be stored here for consistent rendering
   std::vector<muscleAnchor*> m_anchors;

    // Necessary for computations
    double m_restLength;
 
    double m_prevLength;
    
    // So we can get it without passing a dt
    double m_damping;
    
    double m_velocity;
 
    // Should be const for the lifetime of a muscle 
    const btScalar m_dampingCoefficient;
    
    const btScalar m_coefK;

 private: 
    bool invariant(void) const;
};

#endif  // NTRT_MUSCLE2P_H_
