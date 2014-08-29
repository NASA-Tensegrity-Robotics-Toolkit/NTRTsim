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
#if (0)
CordeModel::CordeModel(btVector3 pos1, btVector3 pos2, btQuaternion quat1, btQuaternion quat2, CordeModel::Config& Config) : 
m_config(Config),
    simTime(0.0)
{    
    btVector3 rodLength(pos2 - pos1);
    btVector3 unitLength( rodLength / ((double) m_config.resolution - 1) );
    btVector3 massPos(pos1);
    
    computeConstants(rodLength.length());
    
    double unitMass =  m_config.density * M_PI * pow( m_config.radius, 2) * unitLength.length(); // This might not be the best assumption
    
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
    
    /// Start first step
	stepPrerequisites();
	computeInternalForces();
    
    assert(invariant());
}
#endif // Comment out constructor

CordeModel::CordeModel(std::vector<btVector3>& centerLine, CordeModel::Config& Config) : 
m_config(Config),
    simTime(0.0)
{
	///@todo consider writing interpolation function in case res is higher.
	if (m_config.resolution != centerLine.size())
	{
		throw std::invalid_argument("Resolution does not match specified points");
	}	
	
	CordePositionElement* currentPoint = NULL;
	
	std::size_t n = centerLine.size();
	for (std::size_t i = 0; i < n; i++)
	{
		double unitLength;
		/// @todo - this assumes control points are in centers of mass, and not at ends. Confirm this matches assumptions about anchors and contact.
		if (i != n - 1)
		{	
			unitLength = (centerLine[i+1] - centerLine[i]).length();
			linkLengths.push_back(unitLength);
		}
		else
		{
			unitLength = (centerLine[i] - centerLine[i-1]).length();
		}
		double unitMass = m_config.density * M_PI * pow( m_config.radius, 2) * unitLength;
		currentPoint = new CordePositionElement(centerLine[i], unitMass);
        m_massPoints.push_back(currentPoint);
	}
	
	computeConstants();
	computeCenterlines();
	
	/// Start first step
	stepPrerequisites();
	computeInternalForces();
	
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
    
    m_massPoints.clear();
    m_centerlines.clear();
}

btVector3& CordeModel::getPosition(const std::size_t i) const
{
    if (i >= m_massPoints.size())
    {
        throw std::invalid_argument("Index is greater than size of m_massPoints");
    }
    else
    {
        return m_massPoints[i]->pos;
    }
}

void CordeModel::applyForce(const btVector3& force, const std::size_t segN)
{
    if (segN >= m_massPoints.size())
    {
        throw std::invalid_argument("Index is greater than size of m_massPoints");
    }
    else
    {
        m_massPoints[segN]->applyForce(force);
    }
}

void CordeModel::applyUniformForce(const btVector3& force)
{
    const std::size_t n = m_massPoints.size();
    
    for(std::size_t i = 0; i < n; i++)
    {
        m_massPoints[i]->applyForce(force);
    }
}

void CordeModel::applyUniformAcc(const btVector3& acc)
{
    const std::size_t n = m_massPoints.size();
    
    for(std::size_t i = 0; i < n; i++)
    {
        m_massPoints[i]->applyForce(m_massPoints[i]->mass * acc);
    }
}

void CordeModel::applyVecTorque(const btVector3& tq, const std::size_t segN)
{
	if (segN >= m_centerlines.size())
    {
        throw std::invalid_argument("Index is greater than size of m_centerlines");
    }
    else
    {
        m_centerlines[segN]->appTorques += tq;
    }
}

void CordeModel::applyQuatTorque(const btQuaternion& qtq, const std::size_t segN)
{
	if (segN >= m_centerlines.size())
    {
        throw std::invalid_argument("Index is greater than size of m_centerlines");
    }
    else
    {
        m_centerlines[segN]->tprime += qtq;
    }
}

#if (0)
void CordeModel::step (btScalar dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("Timestep is not positive.");
    }
   
    unconstrainedMotion(dt);
    constrainMotion(dt);
    simTime += dt;
	
    #if (1) ///@todo create a print out function
    if (simTime >= 1.0/1.0)
    {
        size_t n = m_massPoints.size();
        for (std::size_t i = 0; i < n; i++)
        {
            std::cout << "Position " << i << " " << m_massPoints[i]->pos << std::endl
					  << "Velocity " << i << " " << m_massPoints[i]->vel << std::endl
                      << "Force " << i << " " << m_massPoints[i]->force << std::endl;
            if (i < n - 1)
            {
            std::cout << "Quaternion " << i << " " << m_centerlines[i]->q << std::endl
                      << "Qdot " << i << " " << m_centerlines[i]->qdot << std::endl
                      << "Omega " << i << " " << m_centerlines[i]->omega << std::endl
                      << "Force " << i << " " << m_centerlines[i]->tprime << std::endl
                      << "Torque " << i << " " << m_centerlines[i]->torques << std::endl;
            }       
        }
        simTime = 0.0;
    }
    #endif
    
	/// Start next step so other classes can apply external forces.
	stepPrerequisites();
	computeInternalForces();
    
    assert(invariant());
}
#endif


void CordeModel::computeConstants()
{
    assert(computedStiffness.empty());
    
    const double pir2 =  M_PI * pow(m_config.radius, 2);
    
    computedStiffness.push_back( m_config.StretchMod * pir2);
    computedStiffness.push_back( m_config.YoungMod * pir2 / 4.0);
    computedStiffness.push_back( m_config.YoungMod * pir2 / 4.0);
    computedStiffness.push_back( m_config.ShearMod * pir2 / 2.0);
                      
}

void CordeModel::computeCenterlines()
{
	// Ensure all mass points have been created
	assert(m_massPoints.size() == m_config.resolution && m_massPoints.size() == (linkLengths.size() + 1));
	
	std::size_t n = m_massPoints.size() - 1;
	for (std::size_t i = 0; i < n; i++)
	{
		std::vector<btVector3> directorAxes;
		
		double length;
		if (i != n-1)
		{		
			length = (linkLengths[i] + linkLengths[i+1]) / 2.0;
		}
		else
		{
			length = (linkLengths[i-1] + linkLengths[i]) / 2.0;
		}
		
		btVector3 zAxis(0.0, 0.0, 1.0);
		btVector3 axisVec = (m_massPoints[i+1]->pos - m_massPoints[i]->pos).normalize();

		std::cout << zAxis.cross(axisVec).normalize() << std::endl;
		std::cout << acos(zAxis.dot(axisVec)) << std::endl;
		
		btQuaternion currentAngle;
		if (zAxis.dot(axisVec) != 1.0)
		{
			currentAngle.setRotation( zAxis.cross(axisVec).normalize(), acos(zAxis.dot(axisVec)));
		}
		else
		{
			currentAngle.setRotation( zAxis, 0.0);
		}
		
		double mass = m_config.density * length * M_PI * pow(m_config.radius, 2);
		
		btVector3 inertia(mass * (3.0 * pow(length, 2) + pow(m_config.radius, 2)) / 12.0,
							mass * (3.0 * pow(length, 2) + pow(m_config.radius, 2)) / 12.0,
							mass * pow(m_config.radius, 2) / 2.0);
		
		CordeQuaternionElement* nextElement = new CordeQuaternionElement(currentAngle, inertia);
		
		std::cout << nextElement->q << std::endl;
		
		m_centerlines.push_back(nextElement);
		
		if (i != 0)
		{				
			computeQuaternionShapes(i, length);
		}
	}
	
}

/**
 * @todo consider making a reset function for the elements
 */
void CordeModel::stepPrerequisites()
{
    std::size_t n = m_massPoints.size();
	for (std::size_t i = 0; i < n; i++)
    {
        CordePositionElement* r_0 = m_massPoints[i];
        r_0->force.setZero();
        r_0->pos_new = r_0->pos;
        r_0->vel_new = r_0->vel;
    }
    
    n = m_centerlines.size();
	for (std::size_t i = 0; i < n; i++)
    {
        CordeQuaternionElement* q_0 = m_centerlines[i];
        q_0->tprime = btQuaternion(0.0, 0.0, 0.0, 0.0);
        q_0->torques.setZero();
        q_0->appTorques.setZero();
        q_0->q_new = q_0->q;
        q_0->qdot_new = q_0->qdot;
        q_0->omega_new = q_0->omega;
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
        - director[0] * (x1 - x2) * (y1 - y2) ) / ( pow (posNorm, 3) );
        
        /* Quaternion Constraint Z */
        const btScalar quat_cons_z = m_config.ConsSpringConst * linkLengths[i] *
        ( -1.0 * director[0] * (y1 - y2) * (z1 - z2) + director[2] * ( pow( posDiff[0], 2) + pow( posDiff[1], 2) )
        - director[1] * (x1 - x2) * (z1 - z2) ) / ( pow (posNorm, 3) );

        /* Apply X forces */
        r_0->force[0] += -1.0 * (x1 - x2) * (-1.0 * spring_common + diss_common);
        
        r_1->force[0] +=  (x1 - x2) * (-1.0 * spring_common + diss_common);
        
        /* Apply Y forces */
        r_0->force[1] += -1.0 * (y1 - y2) * (-1.0 * spring_common + diss_common);
        
        r_1->force[1] += (y1 - y2) * (-1.0 * spring_common + diss_common);
        
        /* Apply Z Forces */
        r_0->force[2] += -1.0 * (z1 - z2) * (-1.0 * spring_common + diss_common);
        
        r_1->force[2] += (z1 - z2) * (-1.0 * spring_common + diss_common);

        /* Apply constraint equation with boundry conditions */
        /* 8/5/14 - need was confirmed once equations corrected */
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
        quat_0->tprime[0] -= 2.0 * m_config.ConsSpringConst * linkLengths[i]
            * ( q11 + (q13 * posDiff[0] -
            q14 * posDiff[1] - q11 * posDiff[2]) / posNorm);
        
        quat_0->tprime[1] -= 2.0 * m_config.ConsSpringConst * linkLengths[i]
            * ( q12 + (q14 * posDiff[0] +
            q13 * posDiff[1] - q12 * posDiff[2]) / posNorm);
            
        quat_0->tprime[2] -= 2.0 * m_config.ConsSpringConst * linkLengths[i]
            * ( q13 + (q11 * posDiff[0] +
            q12 * posDiff[1] + q13 * posDiff[2]) / posNorm);
            
        quat_0->tprime[3] -= 2.0 * m_config.ConsSpringConst * linkLengths[i]
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
        
        const btScalar u1 = quat_0->qdot[0];
        
        const btScalar k1 = computedStiffness[1];
        const btScalar k2 = computedStiffness[2];
        const btScalar k3 = computedStiffness[3];
        
        const btScalar lj = quaternionShapes[i][0];
        const btScalar mu1 = quaternionShapes[i][1];
        const btScalar mu2 = quaternionShapes[i][2];
        const btScalar mu3 = quaternionShapes[i][3];

        const btScalar c_mu1 = -2.0 * (q11 * q24 + q12 * q23 - q13 * q22 - q14 * q21);
        const btScalar c_mu2 = 2.0 * (q11 * q23 - q12 * q24 - q13 * q21 + q14 * q22);
        const btScalar c_mu3 = -2.0 * (q11 * q22 - q12 * q21 + q13 * q24 - q14 * q23);
          
        /* Bending and torsional stiffness */
        const btScalar stiffness_common = 2.0 / lj;
        
        const btScalar q11_stiffness = stiffness_common * 
        (k1 * q24 * (-1.0 * c_mu1 + lj * mu1) +
         k2 * q23 * (c_mu2 - lj * mu2) +
         k3 * q22 * (-1.0 * c_mu3 + lj * mu3));
         
        const btScalar q12_stiffness = stiffness_common * 
        (k1 * q23 * (-1.0 * c_mu1 + lj * mu1) +
         k2 * q24 * (-1.0 * c_mu2 + lj * mu2) +
         k3 * q21 * (c_mu3 - lj * mu3));
         
        const btScalar q13_stiffness = stiffness_common * 
        (k1 * q22 * (c_mu1 - lj * mu1) +
         k2 * q21 * (-1.0 * c_mu2 + lj * mu2) +
         k3 * q24 * (-1.0 * c_mu3 + lj * mu3));
         
        const btScalar q14_stiffness = stiffness_common * 
        (k1 * q21 * (c_mu1 - lj * mu1) +
         k2 * q22 * (c_mu2 - lj * mu2) +
         k3 * q23 * (c_mu3 - lj * mu3)); 
        
        const btScalar q21_stiffness = stiffness_common *
        (k1 * q14 * (c_mu1  - lj * mu1) +
         k2 * q13 * (-1.0 * c_mu2 + lj * mu2)+
         k3 * q12 * (c_mu3 - lj * mu3));
        
        const btScalar q22_stiffness = stiffness_common *
        (k1 * q13 * (c_mu1 - lj * mu1) + 
         k2 * q14 * (c_mu2 - lj * mu2) +
         k3 * q11 * (-1.0 * c_mu3 + lj * mu3));
         
        const btScalar q23_stiffness = stiffness_common *
        (k1 * q12 * (-1.0 * c_mu1 + lj * mu1) +
         k2 * q11 * (c_mu2 - lj * mu2) +
         k3 * q14 * (c_mu3 - lj * mu3));
         
        const btScalar q24_stiffness = stiffness_common *
        (k1 * q11 * (-1.0 * c_mu1 + lj * mu1) +
         k2 * q12 * (-1.0 * c_mu2 + lj * mu2) +
         k3 * q13 * (-1.0 * c_mu3 + lj * mu3));
         
        /* I apologize for the mess below - the derivatives involved
         * here do not leave a lot of common factors. If you see
         * any nice vector operations I missed, implement them and/or
         * let me know! _Brian
         */
         
        /* Torsional Damping */
        const btScalar damping_common = 4.0 * m_config.gammaR / lj;
        
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
      
        /* Apply torques */ /// @todo - check with theory!
        quat_0->tprime[0] -= q11_stiffness - q11_damping;
       
        quat_1->tprime[0] -= q21_stiffness - q21_damping;
        
        quat_0->tprime[1] -= q12_stiffness - q12_damping;
    
        quat_1->tprime[1] -= q22_stiffness - q22_damping;
        
        quat_0->tprime[2] -= q13_stiffness - q13_damping;
        
        quat_1->tprime[2] -= q23_stiffness - q23_damping;
        
        quat_0->tprime[3] -= q14_stiffness - q14_damping;
        
        quat_1->tprime[3] -= q24_stiffness - q24_damping;

    }
    
    for (std::size_t i = 0; i < m_centerlines.size(); i++)
    {
        /* Transpose quaternion torques into Euclidean torques */
        CordeQuaternionElement* quat_0 = m_centerlines[i];
        quat_0->transposeTorques();
    }
}

void CordeModel::unconstrainedMotion(double dt)
{
    for (std::size_t i = 0; i < m_massPoints.size(); i++)
    {
        CordePositionElement* p_0 = m_massPoints[i];
        
#if (1) 
	   // Eliminate vibrations due to imprecision
		for (std::size_t i = 0; i < 3; i++)
		{
			double a = DBL_EPSILON;
			p_0->force[i] = std::abs(p_0->force[i]) < FLT_EPSILON ? 0.0 : p_0->force[i];
		}
#endif 
        
        // Velocity update - semi-implicit Euler
        p_0->vel_new += dt / p_0->mass * p_0->force;
        // Position update, uses v(t + dt)
        p_0->pos_new += dt * p_0->vel_new;
    }
    for (std::size_t i = 0; i < m_centerlines.size(); i++)
    {   
        CordeQuaternionElement* quat_0 = m_centerlines[i];        
        const btVector3 omega = quat_0->omega;
        // Since I is diagonal, we can use elementwise multiplication of vectors
        quat_0->omega_new += quat_0->inverseInertia * (quat_0->torques - 
            omega.cross(quat_0->computedInertia * omega)) * dt;
        quat_0->updateQDotNew();
        if (quat_0->q.dot(quat_0->qdot_new*dt + quat_0->q) < 0)
        {
            // This'll probably never happen. But if it does and the physics are otherwise mostly reasonable, see
            // http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=9632
            // for solution
            throw std::runtime_error("Tripped quaternion condition.");
        }
        quat_0->q_new = (quat_0->qdot_new*dt + quat_0->q);
        quat_0->q_new /= quat_0->q_new.length();
        if (quat_0->q_new.length() <= 0.99999 || quat_0->q_new.length() >= 1.00001)
        {
            throw std::runtime_error("Tripped quaternion length condition.");
        }
    }
}

/// @todo consider combining with unconstrained motion via a flag
void CordeModel::constrainMotion (double dt)
{
    for (std::size_t i = 0; i < m_massPoints.size(); i++)
    {
        CordePositionElement* p_0 = m_massPoints[i];
        
#if (1) 
	   // Eliminate vibrations due to imprecision
		for (std::size_t i = 0; i < 3; i++)
		{
			double a = DBL_EPSILON;
			p_0->force[i] = std::abs(p_0->force[i]) < FLT_EPSILON ? 0.0 : p_0->force[i];
		}
#endif 
        
        // Velocity update - semi-implicit Euler
        p_0->vel += dt / p_0->mass * p_0->force;
        // Position update, uses v(t + dt)
        p_0->pos += dt * p_0->vel;
    }
    for (std::size_t i = 0; i < m_centerlines.size(); i++)
    {   
        CordeQuaternionElement* quat_0 = m_centerlines[i];        
        const btVector3 omega = quat_0->omega;
        // Since I is diagonal, we can use elementwise multiplication of vectors
        quat_0->omega += quat_0->inverseInertia * (quat_0->torques - 
            omega.cross(quat_0->computedInertia * omega)) * dt;
        quat_0->updateQDot();
        if (quat_0->q.dot(quat_0->qdot*dt + quat_0->q) < 0)
        {
            // This'll probably never happen. But if it does and the physics are otherwise mostly reasonable, see
            // http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=9632
            // for solution
            throw std::runtime_error("Tripped quaternion condition.");
        }
        quat_0->q = (quat_0->qdot*dt + quat_0->q);
        quat_0->q /= quat_0->q.length();
        if (quat_0->q_new.length() <= 0.99999 || quat_0->q_new.length() >= 1.00001)
        {
            throw std::runtime_error("Tripped quaternion length condition.");
        }
    }
}

void CordeModel::computeQuaternionShapes(std::size_t i, double lj)
{
	// If this isn't true, this has been called at a bad spot
	assert(i == m_centerlines.size() - 1 && i != 0);
	assert(lj > 0.0);
	
	CordeQuaternionElement* quat_0 = m_centerlines[i - 1];
	CordeQuaternionElement* quat_1 = m_centerlines[i];
	
	/* Setup Variables */
	const btScalar q11 = quat_0->q[0];
	const btScalar q12 = quat_0->q[1];
	const btScalar q13 = quat_0->q[2];
	const btScalar q14 = quat_0->q[3];
	
	const btScalar q21 = quat_1->q[0];
	const btScalar q22 = quat_1->q[1];
	const btScalar q23 = quat_1->q[2];
	const btScalar q24 = quat_1->q[3];
	
	btScalar c = - 2.0 / lj;
	btScalar mu1 = c * (q11 * q24 + q12 * q23 - q13 * q22 - q14 * q21);
	btScalar mu2 = -1.0 * c * (q11 * q23 - q13 * q21 - q12 * q24 + q14 * q22);
	btScalar mu3 = c * (q11 * q22 - q12 * q21 + q13 * q24 - q14 * q23);
	
	btQuaternion shapeQ(lj, mu1, mu2, mu3);
	quaternionShapes.push_back(shapeQ);
	
	assert(quaternionShapes.size() == m_centerlines.size() - 1);
}

CordeModel::CordePositionElement::CordePositionElement(btVector3 p1, double m) :
	pos(p1),
	pos_new(p1),
	vel(0.0, 0.0, 0.0),
	vel_new(0.0, 0.0, 0.0),
	force(0.0, 0.0, 0.0),
	mass(m)
{
	/// @todo get consistency on what this allowed to be with cordeColliders.cpp
    if (m < 0.0)
    {
        throw std::invalid_argument("Mass is negative.");
    }
}

void CordeModel::CordePositionElement::applyForce(const btVector3& f)
{	
	force += f;
}

CordeModel::CordeQuaternionElement::CordeQuaternionElement(btQuaternion q1, btVector3 inertia) :
    q(q1.normalize()),
    q_new(q),
	qdot(0.0, 0.0, 0.0, 0.0),
	qdot_new(qdot),
    tprime(0.0, 0.0, 0.0, 0.0),
	torques(0.0, 0.0, 0.0),
	omega(0.0, 0.0, 0.0),
	omega_new(omega),
	appTorques(0.0, 0.0, 0.0),
	computedInertia(inertia),
	inverseInertia(1.0/computedInertia[0],
                    1.0/computedInertia[1],
                    1.0/computedInertia[2])
{
    // Can assume if one element is zero, all elements are zero and we've screwed up
    // Should pass automatically based on exceptions in config constructor
    assert(!computedInertia.fuzzyZero());
}

void CordeModel::CordeQuaternionElement::transposeTorques()
{
	torques[0] = 1.0/2.0 * (q[2] * tprime[1] - q[1] * tprime[2] - q[0] * tprime[3] + q[3] * tprime[0]);
    torques[1] = 1.0/2.0 * (q[0] * tprime[2] - q[2] * tprime[0] - q[1] * tprime[3] + q[3] * tprime[1]);
    torques[2] = 1.0/2.0 * (q[1] * tprime[0] - q[0] * tprime[1] - q[2] * tprime[3] + q[3] * tprime[2]);

	torques += appTorques;
#if (1) 
   // Eliminate vibrations due to imprecision
    for (std::size_t i = 0; i < 3; i++)
    {
		double a = DBL_EPSILON;
		torques[i] = std::abs(torques[i]) < FLT_EPSILON ? 0.0 : torques[i];
	}
#endif  
}

// omega holds history.
void CordeModel::CordeQuaternionElement::updateQDot()
{
    qdot[0] = 1.0/2.0 * (q[1] * omega[2] - q[2] * omega[1] + q[3] * omega[0]);
    qdot[1] = 1.0/2.0 * (q[2] * omega[0] - q[0] * omega[2] + q[3] * omega[1]);
    qdot[2] = 1.0/2.0 * (q[0] * omega[1] - q[1] * omega[0] + q[3] * omega[2]);
    qdot[3] = 1.0/2.0 * (-1.0*q[0] * omega[0] - q[1] * omega[1] - q[2] * omega[2]);
}

// omega holds history.
void CordeModel::CordeQuaternionElement::updateQDotNew()
{
    qdot_new[0] = 1.0/2.0 * (q[1] * omega_new[2] - q[2] * omega_new[1] + q[3] * omega_new[0]);
    qdot_new[1] = 1.0/2.0 * (q[2] * omega_new[0] - q[0] * omega_new[2] + q[3] * omega_new[1]);
    qdot_new[2] = 1.0/2.0 * (q[0] * omega_new[1] - q[1] * omega_new[0] + q[3] * omega_new[2]);
    qdot_new[3] = 1.0/2.0 * (-1.0*q[0] * omega_new[0] - q[1] * omega_new[1] - q[2] * omega_new[2]);
}

/// Checks lengths of vectors. @todo add additional invariants
bool CordeModel::invariant()
{
    return (m_massPoints.size() == m_centerlines.size() + 1)
        && (m_centerlines.size() == linkLengths.size())
        && (linkLengths.size() == quaternionShapes.size() + 1)
        && (computedStiffness.size() == 4);
}
