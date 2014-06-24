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
        for (std::size_t i = 0; i < m_massPoints.size(); i++)
        {
            std::cout << "Position " << i << " " << m_massPoints[i]->pos 
                      << "Force " << i << " " << m_massPoints[i]->force << std::endl;
        }
        simTime = 0.0;
    }
    
    assert(invariant());
}

void CordeModel::computeConstants()
{
    computedStiffness.push_back( m_config.StretchMod * M_PI * pow(m_config.radius, 2));
    computedStiffness.push_back( m_config.YoungMod * M_PI * pow(m_config.radius, 2) / 4.0);
    computedStiffness.push_back( m_config.YoungMod * M_PI * pow(m_config.radius, 2) / 4.0);
    computedStiffness.push_back( m_config.ShearMod * M_PI * pow(m_config.radius, 2) / 2.0);
}

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
        
        /* Apply x forces*/
        r_0->force[0] += -1.0 * (x1 - x2) * (spring_common + diss_common) - quat_cons_x;
        
        r_1->force[0] += (x1 - x2) * (spring_common + diss_common) + quat_cons_x;
        
        /* Quaternion Constraint Y */
        const btScalar quat_cons_y = m_config.ConsSpringConst * linkLengths[i] *
        ( -1.0 * director[2] * (y1 - y2) * (z1 - z2) + director[1] * ( pow( posDiff[0], 2) + pow( posDiff[2], 2) )
        - director[0] * (x1 - x2) * (z1 - z2) ) / ( pow (posNorm, 3) );
        
        /* Apply Y forces */
        r_0->force[1] += -1.0 * (y1 - y2) * (spring_common + diss_common) - quat_cons_y;
        
        r_1->force[1] += (y1 - y2) * (spring_common + diss_common) + quat_cons_y;
        
        /* Quaternion Constraint Z */
        const btScalar quat_cons_z = m_config.ConsSpringConst * linkLengths[i] *
        ( -1.0 * director[0] * (y1 - y2) * (z1 - z2) + director[2] * ( pow( posDiff[0], 2) + pow( posDiff[1], 2) )
        - director[1] * (x1 - x2) * (z1 - z2) ) / ( pow (posNorm, 3) );
        
        /* Apply Z Forces */
        r_0->force[2] += -1.0 * (z1 - z2) * (spring_common + diss_common) - quat_cons_z;
        
        r_1->force[2] += (z1 - z2) * (spring_common + diss_common) + quat_cons_z;
        
        /* Torques resulting from quaternion alignment constraints */
        quat_0->tprime[0] += 2.0 * m_config.ConsSpringConst * linkLengths[i]
            * ( q11 * quat_0->q.length2() + (q13 * posDiff[0] -
            q14 * posDiff[1] - q11 * posDiff[2]) / posNorm);
        
        quat_0->tprime[1] += 2.0 * m_config.ConsSpringConst * linkLengths[i]
            * ( q12 * quat_0->q.length2() + (q13 * posDiff[0] +
            q13 * posDiff[1] - q12 * posDiff[2]) / posNorm);
            
        quat_0->tprime[2] += 2.0 * m_config.ConsSpringConst * linkLengths[i]
            * ( q13 * quat_0->q.length2() + (q11 * posDiff[0] +
            q12 * posDiff[1] + q13 * posDiff[2]) / posNorm);
            
        quat_0->tprime[3] += 2.0 * m_config.ConsSpringConst * linkLengths[i]
            * ( q14 * quat_0->q.length2() + (q12 * posDiff[0] -
            q11 * posDiff[1] + q14 * posDiff[2]) / posNorm);
    }
    
    n = m_centerlines.size() - 1;
    
    // Update quaternion elements
	for (std::size_t i = 0; i < n; i++)
    {
        CordeQuaternionElement* quat_0 = m_centerlines[i];
        CordeQuaternionElement* quat_1 = m_centerlines[i + 1];
        
        const btScalar q11 = quat_0->q[0];
        const btScalar q12 = quat_0->q[1];
        const btScalar q13 = quat_0->q[2];
        const btScalar q14 = quat_0->q[3];
        
        const btScalar q21 = quat_1->q[0];
        const btScalar q22 = quat_1->q[1];
        const btScalar q23 = quat_1->q[2];
        const btScalar q24 = quat_1->q[3];
        
        const btScalar k1 = computedStiffness[1];
        const btScalar k2 = computedStiffness[2];
        const btScalar k3 = computedStiffness[3];
        
        /* Bending and torsional stiffness */        
        const btScalar stiffness_common = 4.0 / quaternionShapes[i] *
        pow(quaternionShapes[i] - 1.0, 2);
        
        const btScalar q1_stiffness = stiffness_common * 
        (k1 * q24 * (q11 * q24 + q12 * q23 - q13 * q22 - q14 * q21) +
         k2 * q23 * (q11 * q23 - q12 * q24 - q13 * q21 + q14 * q22) +
         k3 * q22 * (q11 * q22 - q12 * q21 + q13 * q24 - q14 * q23));
         
        const btScalar q2_stiffness = stiffness_common * 
        (k1 * q23 * (q12 * q23 + q11 * q24 - q13 * q22 - q14 * q21) +
         k2 * q24 * (q12 * q24 - q11 * q23 + q13 * q21 - q14 * q24) +
         k3 * q21 * (q12 * q21 - q11 * q22 - q13 * q24 + q14 * q23));
         
        const btScalar q3_stiffness = stiffness_common * 
        (k1 * q22 * (q13 * q22 - q11 * q24 - q12 * q23 + q14 * q21) +
         k2 * q21 * (q13 * q21 - q11 * q23 + q12 * q24 - q14 * q22) +
         k3 * q24 * (q13 * q24 + q11 * q22 - q12 * q21 - q14 * q23));
         
        const btScalar q4_stiffness = stiffness_common * 
        (k1 * q21 * (q14 * q21 - q11 * q24 - q12 * q23 + q13 * q22) +
         k2 * q22 * (q14 * q22 + q11 * q23 - q12 * q24 - q13 * q21) +
         k3 * q23 * (q14 * q23 - q11 * q22 + q12 * q21 - q13 * q24));   
    /*
     * (K1*q14*q21^2 - K1*q11*q21*q24 - K1*q12*q21*q23 + K1*q13*q21*q22
      + K2*q14*q22^2 + K2*q11*q22*q23 - K2*q12*q22*q24 - K2*q13*q21*q22
      + K3*q14*q23^2 - K3*q11*q22*q23 + K3*q12*q21*q23 - K3*q13*q23*q24))/lj
     */
    }
}

void CordeModel::unconstrainedMotion(double dt)
{
    for (std::size_t i = 0; i < m_massPoints.size(); i++)
    {
        CordePositionElement* p_0 = m_massPoints[i];
        // Velocity update - semi-implicit Euler
        p_0->vel = p_0->vel + dt / p_0->mass * p_0->force;
        // Position update, uses v(t + dt)
        p_0->pos = p_0->pos + dt * p_0->vel;
    }
    for (std::size_t i = 0; i < m_centerlines.size(); i++)
    {
        // Need to figure out how to include 4x4 matricies
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

/// Checks lengths of vectors. @todo add additional invariants
bool CordeModel::invariant()
{
    return (m_massPoints.size() == m_centerlines.size() + 1)
        && (m_centerlines.size() == linkLengths.size())
        && (linkLengths.size() == quaternionShapes.size() + 1)
        && (computedStiffness.size() == 4);
}
