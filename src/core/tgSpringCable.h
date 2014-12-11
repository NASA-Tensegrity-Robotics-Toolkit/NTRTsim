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

#ifndef SRC_CORE_TG_SPRING_CABLE_H_
#define SRC_CORE_TG_SPRING_CABLE_H_

/**
 * @file tgSpringCable.h
 * @brief Definitions of class tgSpringCable
 * @author Brian Mirletz, Atil Iscen
 * $Id$
 */

// The C++ Standard Library
#include <vector>

// Forward references
class tgSpringCableAnchor;

/**
 * An abstract base class defining the interface for a spring cable
 * model. This either represents a long elastic member, or a stiff
 * cable connected to a more flexible spring.
 */
class tgSpringCable
{
public: 
    
    /**
     * The only constructor. Takes a list of anchors, a coefficient
     * of stiffness, a coefficent of damping, and optionally the amount
     * of pretension in the cable
     * @param[in] anchors - a list of this spring cable's attachements
     * @param[in] coefK - the stiffness of the spring. Must be positive
     * @param[in] dampingCoefficient - the damping in the spring. Must be non-negative
     * @param[in] pretension - must be small enough to keep the rest length positive
     */
    tgSpringCable( const std::vector<tgSpringCableAnchor*>& anchors,
                double coefK,
                double dampingCoefficient,
                double pretension = 0.0);
    
    /**
     * The virtual destructor. Does nothing
     */
    virtual ~tgSpringCable();
    
    /**
     * The function called by model classes to update this class for
     * a given step
     * @param[in] dt - elapsed time since the last step
     */
    virtual void step(double dt) = 0;
    
    /**
     * Returns m_
     */
    virtual const double getRestLength() const;
    
    virtual void setRestLength( const double newRestLength); 

    virtual const double getActualLength() const = 0;

    virtual const double getTension() const = 0;
    
    virtual const double getCoefK() const
    {
        return m_coefK;
    }
    
    virtual const double getCoefD() const
    {
        return m_dampingCoefficient;
    }
    
    virtual const double getVelocity() const
    {
        return m_velocity;
    }
    
    virtual const double getDamping() const
    {
        return m_damping;
    }
    
    /**
     * Child classes will store their type of anchors, but should
     * always define a way to return a vector of base anchors
     */
    virtual const std::vector<tgSpringCableAnchor*> getAnchors() const = 0;

protected:

    // Necessary for computations
    double m_restLength;
 
    double m_prevLength;
    
    // So we can get it without passing a dt
    double m_damping;
    
    double m_velocity;
 
    // Should be const for the lifetime of a muscle 
    const double m_dampingCoefficient;
    
    const double m_coefK;

};

#endif  // SRC_CORE_TG_SPRING_CABLE_H_
