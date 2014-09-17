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

    const btScalar getActualLength() const;

    const double getTension() const;
    
    const double getCoefK() const
    {
        return m_coefK;
    }
    
    /// @todo is this right? i.e. do we want to be using the material strain change
    /// to determine the control changes our motors can administer?
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
    
    muscleAnchor * anchor1;

    muscleAnchor * anchor2;

    std::string name;

    bool recordHistory;


 private:
    // Necessary for computations
    double m_restLength;
 
    double m_prevLength;
    
    // So we can get it without passing a dt
    double m_damping;
    
    double m_velocity;
 
    // Should be const for the lifetime of a muscle 
    const btScalar m_dampingCoefficient;
    
    const btScalar m_coefK;
 
    bool invariant(void) const;
};

class muscleAnchor
{
public:
    muscleAnchor();

    muscleAnchor(btRigidBody *body, btVector3 pos);
    
    ~muscleAnchor();
    
    btVector3 getWorldPosition();

    // Relative to the body
    btVector3 getRelativePosition();

    btRigidBody * attachedBody;

    // Relative to the body when it is first constructed
    btVector3 attachedRelativeOriginalPosition;

    btScalar height;
 private:
};

#endif  // NTRT_MUSCLE2P_H_
