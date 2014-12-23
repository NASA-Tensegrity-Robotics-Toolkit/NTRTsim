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
 * @file tgCPGStringControl.cpp
 * @brief Implementation of the tgCPGStringControl observer class
 * @author Brian Mirletz
 * @date May 2014
 * $Id$
 */

#include "tgCPGActuatorControl.h"

#include "core/tgSpringCable.h"
#include "core/tgSpringCableAnchor.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgBasicActuator.h"
#include "core/tgBulletSpringCableAnchor.h"
#include "controllers/tgImpedanceController.h"
#include "util/CPGEquations.h"
#include "core/tgCast.h"

// The C++ Standard Library
#include <iostream>
#include <stdexcept>
#include <vector>

tgCPGActuatorControl::tgCPGActuatorControl(const double controlStep) :
m_controlTime(0.0),
m_totalTime(0.0),
m_controlStep(controlStep),
m_commandedTension(0.0),
m_pFromBody(NULL),
m_pToBody(NULL)
{
    if (m_controlStep < 0.0)
    {
        throw std::invalid_argument("Negative control step");
    }
}

tgCPGActuatorControl::~tgCPGActuatorControl()
{
	// We don't own these
	m_pFromBody = NULL;
	m_pToBody = NULL;
}

void tgCPGActuatorControl::onAttach(tgSpringCableActuator& subject)
{
	m_controlLength = subject.getStartLength();
    
    // tgSpringCable doesn't know about bullet anchors, so we have to cast here to get the rigid bodies
	std::vector<const tgBulletSpringCableAnchor*> anchors = 
        tgCast::filter<const tgSpringCableAnchor, const tgBulletSpringCableAnchor>(subject.getSpringCable()->getAnchors());

    std::size_t n = anchors.size();
    assert(n >= 2);
    
	m_pFromBody = anchors[0]->attachedBody;
	m_pToBody   = anchors[n - 1]->attachedBody;
}

void tgCPGActuatorControl::onStep(tgSpringCableActuator& subject, double dt)
{
    m_controlTime += dt;
	m_totalTime += dt;
    /// @todo this fails if its attached to multiple controllers!
    /// is there a way to track _global_ time at this level
    
    // Workaround until we implement PID
    tgBasicActuator& m_sca = *(tgCast::cast<tgSpringCableActuator, tgBasicActuator>(subject));
    
    if (m_controlTime >= m_controlStep)
    {
        
		m_commandedTension = motorControl().control(m_sca, m_controlTime, controlLength(), getCPGValue());

        m_controlTime = 0;
    }
    else
    {
		m_sca.moveMotors(dt);
	}
}

void tgCPGActuatorControl::assignNodeNumber (CPGEquations& CPGSys, array_2D nodeParams)
{
    // Ensure that this hasn't already been assigned
    assert(m_nodeNumber == -1);
    
    m_pCPGSystem = &CPGSys;

    std::vector<double> params (7);
    params[0] = nodeParams[0][0]; // Frequency Offset
    params[1] = nodeParams[0][0]; // Frequency Scale
    params[2] = nodeParams[0][1]; // Radius Offset
    params[3] = nodeParams[0][1]; // Radius Scale
    params[4] = 20.0; // rConst (a constant)
    params[5] = 0.0; // dMin for descending commands
    params[6] = 5.0; // dMax for descending commands
            
    m_nodeNumber = m_pCPGSystem->addNode(params);
}

void
tgCPGActuatorControl::setConnectivity(const std::vector<tgCPGActuatorControl*>& allStrings,
                       array_4D edgeParams) 
{
    assert(m_nodeNumber >= 0);
    
    int muscleSize = edgeParams.shape()[1];
    
    std::vector<int> connectivityList;
    std::vector<double> weights;
    std::vector<double> phases;
    
    // Assuming all coupling is two way, there ought to be a way
    // to search faster than O((2N)^2) since every other
    // string has to call this. Ideas are welcome
    for (int i = 0; i < allStrings.size(); i++)
    {
        if (this != allStrings[i])
        {   
            // Make sure we're dealing with the same system
            assert(m_pCPGSystem == allStrings[i]->getCPGSys());
            
            const btRigidBody* theirFromGroup = allStrings[i]->getFromBody();
            const btRigidBody* theirToGroup = allStrings[i]->getToBody();
            
            // "All to all" connectivity for shared rigid bodies
            if(m_pFromBody == theirFromGroup || 
                m_pToBody == theirToGroup || 
                m_pFromBody == theirToGroup ||
                m_pToBody == theirFromGroup)
            {
                int theirMuscle = allStrings[i]->getNodeNumber();
                // Integer division: -1 is behind, 0 is same row 1 is ahead
                int rp = ((m_nodeNumber - theirMuscle) / muscleSize) + 1;
                int j = m_nodeNumber % muscleSize;
                int k = theirMuscle % muscleSize;
                connectivityList.push_back(theirMuscle);
                // Upper triangular matrix, lower number always goes first
                if(j > k){
                    weights.push_back(edgeParams[rp][k][j][0]);
                    phases.push_back(edgeParams[rp][k][j][1]); 
                }
                else
                {
                    weights.push_back(edgeParams[rp][j][k][0]);
                    phases.push_back(edgeParams[rp][j][k][1]); 
                }
            }
        }
    }
    
    m_pCPGSystem->defineConnections(m_nodeNumber, connectivityList, weights, phases);
}

void tgCPGActuatorControl::setupControl(tgImpedanceController& ipc)
{
    tgBaseCPGNode::setupControl(ipc);
}

void tgCPGActuatorControl::setupControl(tgImpedanceController& ipc,
										double controlLength)
{
	 if (controlLength < 0.0)
    {
        throw std::invalid_argument("Negative control length");
    }
	
	m_controlLength = controlLength;
    tgBaseCPGNode::setupControl(ipc);
}
