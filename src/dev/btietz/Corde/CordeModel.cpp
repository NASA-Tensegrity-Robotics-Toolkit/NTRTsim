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

// This library
#include "tgcreator/tgUtil.h"

// The C++ Standard Library
#include <stdexcept>

CordeModel::Config::Config(const std::size_t res,
                            const double r, const double d,
                            const double ym, const double shm,
                            const double stm, const double csc,
                            const double gt, const double gr) :
    resolution(res),
    radius(r),
    density(d),
    YoungMod(ym),
    ShearMod(shm),
    StretchMod(stm),
    ConsSpringConst(csc),
    gammaT(gt),
    gammaR(gr)
{
    if (r <= 0.0)
    {
        throw std::invalid_argument("Corde string radius is not positive.");
    }
    else if (d <= 0.0)
    {
        throw std::invalid_argument("Corde String density is not positive.");
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

CordeModel::CordeModel(btVector3 pos1, btVector3 pos2, btQuaternion quat1, btQuaternion quat2, CordeModel::Config& Config) : 
m_config(Config),
    simTime(0.0)
{
	computeConstants();
    
    btVector3 rodLength(pos2 - pos1);
    btVector3 unitLength( rodLength / ((double) m_config.resolution - 1) );
    btVector3 massPos(pos1);
    
    double unitMass =  m_config.density * M_PI * pow( m_config.radius, 2) * unitLength.length();
    
    CordePositionElement* currentPoint = new CordePositionElement(massPos, unitMass);
    
    m_massPoints.push_back(currentPoint);
    
    std::cout << massPos << std::endl;
    // Setup mass elements
    for (std::size_t i = 1; i < m_config.resolution; i++)
    {
                
        massPos += unitLength;
        currentPoint = new CordePositionElement(massPos, unitMass);
        m_massPoints.push_back(currentPoint);
        // Introduce stretch
        linkLengths.push_back(unitLength.length() * 1.0);
        std::cout << massPos << " " << unitMass << std::endl;
    }
    
    CordeQuaternionElement* currentAngle = new CordeQuaternionElement(quat1);
    m_centerlines.push_back(currentAngle);
    
    std::size_t n = m_config.resolution - 1;
    for (std::size_t i = 1; i < n; i++)
    {
        currentAngle = new CordeQuaternionElement(quat1.slerp(quat2, (double) i / (double) n) );
        m_centerlines.push_back(currentAngle);
        std::cout << currentAngle->q << std::endl;
        quaternionShapes.push_back(unitLength.length());
    }
    
    assert(invariant());
}

CordeModel::~CordeModel()
{
	for (std::size_t i = 0; i < m_massPoints.size(); i++)
    {
        delete m_massPoints[i];
    }
    for (std::size_t i = 0; i < m_centerlines.size(); i++)
    {
        delete m_centerlines[i];
    }
}

void CordeModel::step (btScalar dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("Timestep is not positive.");
    }
    
    stepPrerequisites();
	computeInternalForces();
    unconstrainedMotion(dt);
    simTime += dt;
    if (simTime >= .01)
    {
        size_t n = m_massPoints.size();
        for (std::size_t i = 0; i < n; i++)
        {
            std::cout << "Position " << i << " " << m_massPoints[i]->pos << std::endl
                      << "Force " << i << " " << m_massPoints[i]->force << std::endl;
            if (i < n - 1)
            {
            std::cout << "Quaternion " << i << " " << m_centerlines[i]->q << std::endl
                      << "Qdot " << i << " " << m_centerlines[i]->qdot << std::endl
                      << "Force " << i << " " << m_centerlines[i]->tprime << std::endl
                      << "Torque " << i << " " << m_centerlines[i]->torques << std::endl;
            }       
        }
        simTime = 0.0;
    }
    
    assert(invariant());
}

void CordeModel::computeConstants()
{
    assert(computedStiffness.empty());
    
    const double pir2 =  M_PI * pow(m_config.radius, 2);
    
    computedStiffness.push_back( m_config.StretchMod * pir2);
    computedStiffness.push_back( m_config.YoungMod * pir2 / 4.0);
    computedStiffness.push_back( m_config.YoungMod * pir2 / 4.0);
    computedStiffness.push_back( m_config.ShearMod * pir2 / 2.0);
    
    /* Could probably do this in constructor directly, but easier
     * here since pir2 is already computed
     */
    computedInertia.setValue(m_config.density * pir2 / 4.0, 
                     m_config.density * pir2 / 4.0,
                     m_config.density * pir2 / 2.0);
    
    // Can assume if one element is zero, all elements are zero and we've screwed up
    // Should pass automatically based on exceptions in config constructor
    assert(!computedInertia.fuzzyZero());
    
    inverseInertia.setValue(1.0/computedInertia[0],
                            1.0/computedInertia[1],
                            1.0/computedInertia[2]);
                      
}

/**
 * @todo consider making a reset function for the elements
 */
void CordeModel::stepPrerequisites()
{
    std::size_t n = m_massPoints.size();
	for (std::size_t i = 0; i < n - 1; i++)
    {
        CordePositionElement* r_0 = m_massPoints[i];
        r_0->force.setZero();
    }
    
    n = m_centerlines.size();
	for (std::size_t i = 0; i < n - 1; i++)
    {
        CordeQuaternionElement* q_0 = m_centerlines[i];
        q_0->tprime = btQuaternion(0.0, 0.0, 0.0, 0.0);
        q_0->torques.setZero();
    }
}

void CordeModel::computeInternalForces()
{
    std::size_t n = m_massPoints.size() - 1;
    
    // Update position elements
	for (std::size_t i = 0; i < n; i++)
    {
        CordePositionElement* r_0 = m_massPoints[i];
        CordePositionElement* r_1 = m_massPoints[i + 1];
        
        CordeQuaternionElement* quat_0 = m_centerlines[i];
        
        // Get position elements in standard variable names
        const btScalar x1 = r_0->pos[0];
        const btScalar y1 = r_0->pos[1];
        const btScalar z1 = r_0->pos[2];
        
        const btScalar x2 = r_1->pos[0];
        const btScalar y2 = r_1->pos[1];
        const btScalar z2 = r_1->pos[2];
        
        // Same for quaternion elements
        const btScalar q11 = quat_0->q[0];
        const btScalar q12 = quat_0->q[1];
        const btScalar q13 = quat_0->q[2];
        const btScalar q14 = quat_0->q[3];
        
        // Setup common factors
        const btVector3 posDiff = r_0->pos - r_1->pos;
        const btVector3 velDiff = r_0->vel - r_1->vel;
        const btScalar posNorm   = posDiff.length();
        const btScalar posNorm_2 = posDiff.length2();
        const btVector3 director( (2.0 * (q11 * q13 + q12 * q14)),
                        (2.0 * (q12 * q13 - q11 * q14)),
           ( -1.0 * q11 * q11 - q12 * q12 + q13 * q13 + q14 * q14));
        
        // Sum Forces, have to split it out into components due to
        // derivatives of energy quantaties

        // Spring common
        const btScalar spring_common = computedStiffness[0] * 
            (linkLengths[i] - posNorm) / (linkLengths[i] * posNorm);
        
        const btScalar diss_common = m_config.gammaT *
                        posNorm_2 * posDiff.dot(velDiff) / pow (linkLengths[i] , 5);
        
        /* Quaternion Constraint X */
        const btScalar quat_cons_x = m_config.ConsSpringConst * linkLengths[i] *
        ( director[2] * (x1 - x2) * (z1 - z2) - director[0] * ( pow( posDiff[1], 2) + pow( posDiff[2], 2) )
        + director[1] * (x1 - x2) * (y1 - y2) ) / ( pow (posNorm, 3) );
        
        /* Quaternion Constraint Y */
        const btScalar quat_cons_y = m_config.ConsSpringConst * linkLengths[i] *
        ( -1.0 * director[2] * (y1 - y2) * (z1 - z2) + director[1] * ( pow( posDiff[0], 2) + pow( posDiff[2], 2) )
        - director[0] * (x1 - x2) * (z1 - z2) ) / ( pow (posNorm, 3) );
        
        /* Quaternion Constraint Z */
        const btScalar quat_cons_z = m_config.ConsSpringConst * linkLengths[i] *
        ( -1.0 * director[0] * (y1 - y2) * (z1 - z2) + director[2] * ( pow( posDiff[0], 2) + pow( posDiff[1], 2) )
        - director[1] * (x1 - x2) * (z1 - z2) ) / ( pow (posNorm, 3) );
        
        r_0->force[0] += -1.0 * (x1 - x2) * (spring_common + diss_common);
        
        r_1->force[0] += (x1 - x2) * (spring_common + diss_common);
        
        /* Apply Y forces */
        r_0->force[1] += -1.0 * (y1 - y2) * (spring_common + diss_common);
        
        r_1->force[1] += (y1 - y2) * (spring_common + diss_common);
        
        /* Apply Z Forces */
        r_0->force[2] += -1.0 * (z1 - z2) * (spring_common + diss_common);
        
        r_1->force[2] += (z1 - z2) * (spring_common + diss_common);

        /* Apply constraint equation with boundry conditions */
        if (i == 0)
        {
           r_1->force[0] += quat_cons_x;
            
            
           r_1->force[1] += quat_cons_y;
            
            
           r_1->force[2] += quat_cons_z; 
        }
        else if (i == n - 1)
        {
            r_0->force[0] -= quat_cons_x;

            r_0->force[1] -= quat_cons_y;
            
            r_0->force[2] -= quat_cons_z;       
        }
        else
        {
            r_0->force[0] -= quat_cons_x;
            r_1->force[0] += quat_cons_x;
            
            r_0->force[1] -= quat_cons_y;
            r_1->force[1] += quat_cons_y;
            
            r_0->force[2] -= quat_cons_z;
            r_1->force[2] += quat_cons_z;            
        }

#if (0) // Original derivation
        /* Torques resulting from quaternion alignment constraints */
        quat_0->tprime[0] += 2.0 * m_config.ConsSpringConst * linkLengths[i]
            * ( q11 * quat_0->q.length2() + (q13 * posDiff[0] -
            q14 * posDiff[1] - q11 * posDiff[2]) / posNorm);
        
        quat_0->tprime[1] += 2.0 * m_config.ConsSpringConst * linkLengths[i]
            * ( q12 * quat_0->q.length2() + (q14 * posDiff[0] +
            q13 * posDiff[1] - q12 * posDiff[2]) / posNorm);
            
        quat_0->tprime[2] += 2.0 * m_config.ConsSpringConst * linkLengths[i]
            * ( q13 * quat_0->q.length2() + (q11 * posDiff[0] +
            q12 * posDiff[1] + q13 * posDiff[2]) / posNorm);
            
        quat_0->tprime[3] += 2.0 * m_config.ConsSpringConst * linkLengths[i]
            * ( q14 * quat_0->q.length2() + (q12 * posDiff[0] -
            q11 * posDiff[1] + q14 * posDiff[2]) / posNorm);
#else // quat_0->q.length2() should always be 1, but sometimes numerical precision renders it slightly greater
        // The simulation is much more stable if we just assume its one.
        quat_0->tprime[0] += 2.0 * m_config.ConsSpringConst * linkLengths[i]
            * ( q11 + (q13 * posDiff[0] -
            q14 * posDiff[1] - q11 * posDiff[2]) / posNorm);
        
        quat_0->tprime[1] += 2.0 * m_config.ConsSpringConst * linkLengths[i]
            * ( q12 + (q14 * posDiff[0] +
            q13 * posDiff[1] - q12 * posDiff[2]) / posNorm);
            
        quat_0->tprime[2] += 2.0 * m_config.ConsSpringConst * linkLengths[i]
            * ( q13 + (q11 * posDiff[0] +
            q12 * posDiff[1] + q13 * posDiff[2]) / posNorm);
            
        quat_0->tprime[3] += 2.0 * m_config.ConsSpringConst * linkLengths[i]
            * ( q14 + (q12 * posDiff[0] -
            q11 * posDiff[1] + q14 * posDiff[2]) / posNorm);
#endif
    }
    
    n = m_centerlines.size() - 1;
    
    // Update quaternion elements
	for (std::size_t i = 0; i < n; i++)
    {
        CordeQuaternionElement* quat_0 = m_centerlines[i];
        CordeQuaternionElement* quat_1 = m_centerlines[i + 1];
        
        /* Setup Variables */
        const btScalar q11 = quat_0->q[0];
        const btScalar q12 = quat_0->q[1];
        const btScalar q13 = quat_0->q[2];
        const btScalar q14 = quat_0->q[3];
        
        const btScalar q21 = quat_1->q[0];
        const btScalar q22 = quat_1->q[1];
        const btScalar q23 = quat_1->q[2];
        const btScalar q24 = quat_1->q[3];
        
        const btScalar qdot11 = quat_0->qdot[0];
        const btScalar qdot12 = quat_0->qdot[1];
        const btScalar qdot13 = quat_0->qdot[2];
        const btScalar qdot14 = quat_0->qdot[3];
        
        const btScalar qdot21 = quat_1->qdot[0];
        const btScalar qdot22 = quat_1->qdot[1];
        const btScalar qdot23 = quat_1->qdot[2];
        const btScalar qdot24 = quat_1->qdot[3];
        
        const btScalar k1 = computedStiffness[1];
        const btScalar k2 = computedStiffness[2];
        const btScalar k3 = computedStiffness[3];
        
        /* I apologize for the mess below - the derivatives involved
         * here do not leave a lot of common factors. If you see
         * any nice vector operations I missed, implement them and/or
         * let me know! _Brian
         */
        
        /* Bending and torsional stiffness */        
        const btScalar stiffness_common = 4.0 / quaternionShapes[i] *
        pow(quaternionShapes[i] - 1.0, 2);
        
        const btScalar q11_stiffness = stiffness_common * 
        (k1 * q24 * (q11 * q24 + q12 * q23 - q13 * q22 - q14 * q21) +
         k2 * q23 * (q11 * q23 - q12 * q24 - q13 * q21 + q14 * q22) +
         k3 * q22 * (q11 * q22 - q12 * q21 + q13 * q24 - q14 * q23));
         
        const btScalar q12_stiffness = stiffness_common * 
        (k1 * q23 * (q12 * q23 + q11 * q24 - q13 * q22 - q14 * q21) +
         k2 * q24 * (q12 * q24 - q11 * q23 + q13 * q21 - q14 * q22) +
         k3 * q21 * (q12 * q21 - q11 * q22 - q13 * q24 + q14 * q23));
         
        const btScalar q13_stiffness = stiffness_common * 
        (k1 * q22 * (q13 * q22 - q11 * q24 - q12 * q23 + q14 * q21) +
         k2 * q21 * (q13 * q21 - q11 * q23 + q12 * q24 - q14 * q22) +
         k3 * q24 * (q13 * q24 + q11 * q22 - q12 * q21 - q14 * q23));
         
        const btScalar q14_stiffness = stiffness_common * 
        (k1 * q21 * (q14 * q21 - q11 * q24 - q12 * q23 + q13 * q22) +
         k2 * q22 * (q14 * q22 + q11 * q23 - q12 * q24 - q13 * q21) +
         k3 * q23 * (q14 * q23 - q11 * q22 + q12 * q21 - q13 * q24));   
        
        const btScalar q21_stiffness = stiffness_common *
        (k1 * q14 * (q14 * q21 - q11 * q24 - q12 * q23 + q13 * q22) +
         k2 * q13 * (q13 * q21 - q11 * q23 + q12 * q24 - q14 * q22) +
         k3 * q12 * (q12 * q21 - q11 * q22 + q14 * q23 - q13 * q24));
        
        const btScalar q22_stiffness = stiffness_common *
        (k1 * q13 * (q13 * q22 - q11 * q24 - q12 * q23 + q14 * q21) + 
         k2 * q14 * (q14 * q22 + q11 * q23 - q12 * q24 - q13 * q21) +
         k3 * q11 * (q11 * q22 - q12 * q21 + q13 * q24 - q14 * q23));
         
        const btScalar q23_stiffness = stiffness_common *
        (k1 * q12 * (q12 * q23 + q11 * q24 - q13 * q22 - q14 * q21) +
         k2 * q11 * (q11 * q23 - q13 * q21 - q12 * q24 + q14 * q22) +
         k3 * q14 * (q14 * q23 - q11 * q22 + q12 * q21 - q13 * q24));
         
        const btScalar q24_stiffness = stiffness_common *
        (k1 * q11 * (q11 * q24 + q12 * q23 - q13 * q22 - q14 * q21) +
         k2 * q12 * (q12 * q24 - q11 * q23 + q13 * q21 - q14 * q22) +
         k3 * q13 * (q13 * q24 + q11 * q22 - q12 * q21 - q14 * q23));
         
        /* Torsional Damping */
        const btScalar damping_common = 4.0 * m_config.gammaR / quaternionShapes[i];
        
        const btScalar q11_damping = damping_common *
        (q12 * (q12 * qdot11 - q11 * qdot12 + q21 * qdot22 - q22 * qdot21 - q23 * qdot24 + q24 * qdot23) +
         q13 * (q13 * qdot11 - q11 * qdot13 + q21 * qdot23 + q22 * qdot24 - q23 * qdot21 - q24 * qdot22) +
         q14 * (q14 * qdot11 - q11 * qdot14 + q21 * qdot24 - q22 * qdot23 + q23 * qdot22 - q24 * qdot21));
         
        const btScalar q12_damping = damping_common *
        (q11 * (q11 * qdot12 - q12 * qdot11 - q21 * qdot22 + q22 * qdot21 + q23 * qdot24 - q24 * qdot23) +
         q13 * (q13 * qdot12 - q13 * qdot13 - q21 * qdot24 + q22 * qdot23 - q23 * qdot22 + q24 * qdot21) + 
         q14 * (q14 * qdot12 - q14 * qdot14 + q21 * qdot23 + q22 * qdot24 - q23 * qdot21 - q24 * qdot22));
         
        const btScalar q13_damping = damping_common * 
        (q11 * (q11 * qdot13 - q13 * qdot11 - q21 * qdot23 - q22 * qdot24 + q23 * qdot21 + q24 * qdot22) +
         q12 * (q12 * qdot13 - q13 * qdot12 + q21 * qdot24 - q22 * qdot23 + q23 * qdot22 - q24 * qdot21) +
         q14 * (q14 * qdot13 - q13 * qdot14 - q21 * qdot22 + q22 * qdot21 + q23 * qdot24 - q24 * qdot23));
         
        const btScalar q14_damping = damping_common *
        (q11 * (q11 * qdot14 - q14 * qdot11 - q21 * qdot24 + q22 * qdot23 - q23 * qdot22 + q24 * qdot21) +
         q12 * (q12 * qdot14 - q14 * qdot12 - q21 * qdot23 - q22 * qdot24 + q23 * qdot21 + q24 * qdot22) +
         q13 * (q13 * qdot14 - q14 * qdot13 + q21 * qdot22 - q22 * qdot21 - q23 * qdot24 + q24 * qdot23));
        
        const btScalar q21_damping = damping_common *
        (q22 * (q22 * qdot21 + q11 * qdot12 - q12 * qdot11 - q13 * qdot14 + q14 * qdot13 - q21 * qdot22) +
         q23 * (q23 * qdot21 + q11 * qdot13 + q12 * qdot14 - q13 * qdot11 - q14 * qdot12 - q21 * qdot23) +
         q24 * (q24 * qdot21 + q11 * qdot14 - q12 * qdot13 + q13 * qdot12 - q14 * qdot11 - q21 * qdot24));
         
        const btScalar q22_damping = damping_common *
        (q21 * (q21 * qdot22 - q11 * qdot12 + q12 * qdot11 + q13 * qdot14 - q14 * qdot13 - q22 * qdot21) +
         q23 * (q23 * qdot22 - q11 * qdot14 + q12 * qdot13 - q13 * qdot12 + q14 * qdot11 - q22 * qdot23) +
         q24 * (q24 * qdot22 + q11 * qdot13 + q12 * qdot14 - q13 * qdot11 - q14 * qdot12 - q22 * qdot24));
         
        const btScalar q23_damping = damping_common *
        (q21 * (q21 * qdot23 - q11 * qdot13 + q13 * qdot11 - q12 * qdot14 + q14 * qdot12 - q23 * qdot21) +
         q22 * (q22 * qdot23 + q11 * qdot14 - q12 * qdot13 + q13 * qdot12 - q14 * qdot11 - q22 * qdot22) +
         q24 * (q24 * qdot23 - q11 * qdot12 + q12 * qdot11 + q13 * qdot14 - q14 * qdot13 - q23 * qdot24));
         
        const btScalar q24_damping = damping_common *
        (q21 * (q21 * qdot24 - q11 * qdot14 + q12 * qdot13 - q13 * qdot12 + q14 * qdot11 - q24 * qdot21) +
         q22 * (q21 * qdot24 - q11 * qdot13 - q12 * qdot14 + q13 * qdot11 + q14 * qdot12 - q24 * qdot22) +
         q23 * (q23 * qdot24 + q11 * qdot12 - q12 * qdot11 - q13 * qdot14 + q14 * qdot13 - q24 * qdot23));
      
        /* Apply torques */ /// @todo double check the sign convention. Looks good numerically.q
        quat_0->tprime[0] += q11_stiffness + q11_damping;
        
        quat_1->tprime[0] += q21_stiffness + q21_damping;
        
        quat_0->tprime[1] += q12_stiffness + q12_damping;
    
        quat_1->tprime[1] += q22_stiffness + q22_damping;
        
        quat_0->tprime[2] += q13_stiffness + q13_damping;
        
        quat_1->tprime[2] += q23_stiffness + q23_damping;
        
        quat_0->tprime[3] += q14_stiffness + q14_damping;
        
        quat_1->tprime[3] += q24_stiffness + q24_damping;

    }
}

void CordeModel::unconstrainedMotion(double dt)
{
    for (std::size_t i = 0; i < m_massPoints.size(); i++)
    {
        CordePositionElement* p_0 = m_massPoints[i];
        // Velocity update - semi-implicit Euler
        p_0->vel += dt / p_0->mass * p_0->force;
        // Position update, uses v(t + dt)
        p_0->pos += dt * p_0->vel;
    }
    for (std::size_t i = 0; i < m_centerlines.size(); i++)
    {
        /* Transpose quaternion torques into Euclidean torques */
        CordeQuaternionElement* quat_0 = m_centerlines[i];
        quat_0->transposeTorques();
        
        const btVector3 omega = quat_0->omega;
        // Since I is diagonal, we can use elementwise multiplication of vectors
        quat_0->omega += inverseInertia * (quat_0->torques - 
            omega.cross(computedInertia * omega)) * dt;
        quat_0->updateQDot();
        quat_0->q = (quat_0->qdot*dt + quat_0->q).normalize();
    }
}

CordeModel::CordePositionElement::CordePositionElement(btVector3 p1, double m) :
	pos(p1),
	vel(0.0, 0.0, 0.0),
	force(0.0, 0.0, 0.0),
	mass(m)
{
    if (m < 0.0)
    {
        throw std::invalid_argument("Mass is negative.");
    }
}

CordeModel::CordeQuaternionElement::CordeQuaternionElement(btQuaternion q1) :
    q(q1.normalize()),
	qdot(0.0, 0.0, 0.0, 0.0),
    tprime(0.0, 0.0, 0.0, 0.0),
	torques(0.0, 0.0, 0.0),
	omega(0.0, 0.0, 0.0)
{
    
}

void CordeModel::CordeQuaternionElement::transposeTorques()
{
    torques[0] += 1.0/2.0 * (q[0] * tprime[2] - q[2] * tprime[0] - q[1] * tprime[3] + q[3] * tprime[1]);
    torques[1] += 1.0/2.0 * (q[1] * tprime[0] - q[0] * tprime[1] - q[2] * tprime[3] + q[3] * tprime[2]);
    torques[2] += 1.0/2.0 * (q[0] * tprime[0] + q[1] * tprime[1] + q[2] * tprime[2] + q[3] * tprime[3]);
}

void CordeModel::CordeQuaternionElement::updateQDot()
{
    qdot[0] = 1.0/2.0 * (q[0] * omega[2] + q[1] * omega[1] - q[2] * omega[0]);
    qdot[1] = 1.0/2.0 * (q[1] * omega[2] - q[0] * omega[1] + q[3] * omega[0]);
    qdot[2] = 1.0/2.0 * (q[0] * omega[0] + q[2] * omega[2] + q[3] * omega[1]);
    qdot[3] = 1.0/2.0 * (q[3] * omega[2] - q[2] * omega[1] - q[1] * omega[0]);
}

/// Checks lengths of vectors. @todo add additional invariants
bool CordeModel::invariant()
{
    return (m_massPoints.size() == m_centerlines.size() + 1)
        && (m_centerlines.size() == linkLengths.size())
        && (linkLengths.size() == quaternionShapes.size() + 1)
        && (computedStiffness.size() == 4);
}
