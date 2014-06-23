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
 * @file CordeModel.cpp
 * @brief Defines structure for the Corde softbody String Model
 * @author Brian Mirletz
 * $Id$
 */

// This module
#include "CordeModel.h"

// The C++ Standard Library
#include <stdexcept>

CordeModel::Config::Config(const double r, const double d,
                            const double ym, const double shm,
                            const double stm, const double csc,
                            const double gt, const double gr) :
    radius(r),
    density(d),
    YoungMod(ym),
    ShearMod(shm),
    StretchMod(stm),
    ConsSpringConst(csc),
    gammaT(gt),
    gammaR(gr)
{
    if (r < 0.0)
    {
        throw std::invalid_argument("Corde string radius is negative.");
    }
    else if (d < 0.0)
    {
        throw std::invalid_argument("Corde String density is negative.");
    }
    else if (ym < 0.0)
    {
        throw std::invalid_argument("String Young's Modulus is negative.");
    }
    else if (shm < 0.0)
    {
        throw std::invalid_argument("Shear Modulus is negative.");
    }
    else if (stm < 0.0)
    {
        throw std::invalid_argument("Stretch Modulus is negative.");
    }
    else if (csc < 0.0)
    {
        throw std::invalid_argument("Spring Constant is negative.");
    }
    else if (gt < 0.0)
    {
        throw std::invalid_argument("Damping Constant (position) is negative.");
    }
    else if (gr < 0.0)
    {
        throw std::invalid_argument("Damping Constant (rotation) is negative.");
    }
}

CordeModel::CordeModel(btVector3 pos1, btVector3 pos2, CordeModel::Config& Config) : 
m_config(Config)
{
	
}

CordeModel::~CordeModel()
{
	
}

void CordeModel::step (btScalar dt)
{
    stepPrerequisites();
	computeInternalForces();
}

void CordeModel::computeConstants()
{
    
}

void CordeModel::stepPrerequisites()
{
    std::size_t n = m_massPoints.size();
	for (std::size_t i = 0; i < n - 1; i++)
    {
        CordePositionElement* r_0 = m_massPoints[i];
        r_0->force.setZero();
    }
    
    n = m_Centerlines.size();
	for (std::size_t i = 0; i < n - 1; i++)
    {
        CordeQuaternionElement* q_0 = m_Centerlines[i];
        q_0->tprime = btQuaternion(0.0, 0.0, 0.0, 0.0);
        q_0->torques.setZero();
    }
}

void CordeModel::computeInternalForces()
{
    std::size_t n = m_massPoints.size();
	for (std::size_t i = 0; i < n - 1; i++)
    {
        CordePositionElement* r_0 = m_massPoints[i];
        CordePositionElement* r_1 = m_massPoints[i + 1];
        
        CordeQuaternionElement* quat_0 = m_Centerlines[i];
        CordeQuaternionElement* quat_1 = m_Centerlines[i + 1];
        
        // Get position elements in standard variable names
        const btScalar x1 = r_0->pos[0];
        const btScalar y1 = r_0->pos[1];
        const btScalar z1 = r_0->pos[2];
        
        const btScalar x2 = r_1->pos[0];
        const btScalar y2 = r_1->pos[1];
        const btScalar z2 = r_1->pos[2];
        
        // Same for quaternion elements
        const btScalar q1_1 = quat_0->q[0];
        const btScalar q1_2 = quat_0->q[1];
        const btScalar q1_3 = quat_0->q[2];
        const btScalar q1_4 = quat_0->q[3];
        
        const btScalar q2_1 = quat_1->q[0];
        const btScalar q2_2 = quat_1->q[1];
        const btScalar q2_3 = quat_1->q[2];
        const btScalar q2_4 = quat_1->q[3];
        
        // Setup common factors
        const btVector3 posDiff = r_0->pos - r_1->pos;
        const btVector3 velDiff = r_0->vel - r_1->vel;
        const btScalar posNorm   = posDiff.length();
        const btScalar posNorm_2 = posDiff.length2();
        const btVector3 director( (2.0 * (q1_1 * q1_3 + q1_2 * q1_4)),
                                  (2.0 * (q1_2 * q1_3 - q1_1 * q1_4)),
           ( -1.0 * q1_1 * q1_1 - q1_2 * q1_2 + q1_3 * q1_3 + q1_4 * q1_4));
        
        // Sum X Forces:
        
        // Spring Constraint
        const btScalar spring_cons_x = computedStiffness[0] * 
            (linkLengths[i] - posNorm) * (x1 - x2) / (linkLengths[i] * posNorm);
        
        // Quaternion Constraint
        const btScalar quat_cons_x = m_config.ConsSpringConst * linkLengths[i] *
        ( director[2] * (x1 - x2) * (z1 - z2) - director[0] * ( pow( posDiff[1], 2) + pow( posDiff[2], 2) )
        + director[1] * (x1 - x2) * (y1 - y2) ) / ( pow (posNorm, 3) );
        
        const btScalar diss_energy_x = m_config.gammaT * (x1 - x2) * 
                        posNorm_2 * posDiff.dot(velDiff) / pow (linkLengths[i] , 5);
                            
        
        r_0->force[0] += -1.0 * spring_cons_x - quat_cons_x;
             
        r_1->force[0] += spring_cons_x + quat_cons_x;
    }
}

