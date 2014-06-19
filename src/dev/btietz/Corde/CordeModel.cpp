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

// Bullet Linear Algebra
#include "CordeModel.h"

CordeModel::CordeModel(btVector3 pos1, btVector3 pos2)
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
        const CordePositionElement* r_0 = m_massPoints[i];
        const CordePositionElement* r_1 = m_massPoints[i + 1];
        
        const CordeQuaternionElement* quat_0 = m_Centerlines[i];
        const CordeQuaternionElement* quat_1 = m_Centerlines[i + 1];
        
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
        btScalar posNorm_2;
    }
}

