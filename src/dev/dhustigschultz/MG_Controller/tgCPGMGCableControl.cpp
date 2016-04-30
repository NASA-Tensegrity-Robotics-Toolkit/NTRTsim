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

#include "tgCPGMGCableControl.h"

#include "core/tgSpringCableActuator.h"
#include "core/tgBulletSpringCableAnchor.h"
#include "core/tgBasicActuator.h"
#include "controllers/tgImpedanceController.h"
#include "controllers/tgPIDController.h"
#include "core/tgCast.h"
#include "util/CPGEquations.h"
#include "dev/CPG_feedback/CPGEquationsFB.h"

// The C++ Standard Library
#include <iostream>
#include <stdexcept>
#include <vector>

tgCPGMGCableControl::tgCPGMGCableControl(tgPIDController::Config pid_config, const double controlStep) :
m_config(pid_config),
tgCPGMGActuatorControl(controlStep),
m_PID(NULL),
usePID(true)
{
    if (m_controlStep < 0.0)
    {
        throw std::invalid_argument("Negative control step");
    }
}

tgCPGMGCableControl::~tgCPGMGCableControl()
{
    delete m_PID;
}

void tgCPGMGCableControl::onSetup(tgSpringCableActuator& subject)
{
    m_PID = new tgPIDController(&subject, m_config);
    
    tgBasicActuator* basicAct =  tgCast::cast<tgSpringCableActuator, tgBasicActuator>(&subject);
    
    if (basicAct != NULL)
    {
        usePID = false;
    }
}

void tgCPGMGCableControl::onStep(tgSpringCableActuator& subject, double dt)
{
    assert(&subject == m_PID->getControllable());
    
    m_controlTime += dt;
	m_totalTime += dt;

    if (m_controlTime >= m_controlStep)
    {
        if (usePID)
        {
            m_commandedTension = motorControl().control(*m_PID, dt, controlLength(), getCPGValue());
        }
        else
        {
            tgBasicActuator* basicAct =  tgCast::cast<tgSpringCableActuator, tgBasicActuator>(&subject);
            m_commandedTension = motorControl().control(*basicAct, dt, controlLength(), getCPGValue());
        }
        m_controlTime = 0;
    }
    else
    {
		const double currentTension = subject.getTension();
        if(usePID)
        {
            m_PID->control(dt, m_commandedTension, currentTension);
        }
        else
        {
            tgBasicActuator* basicAct =  tgCast::cast<tgSpringCableActuator, tgBasicActuator>(&subject);
            basicAct->moveMotors(dt);
        }
	}
}

void tgCPGMGCableControl::assignNodeNumberFB (CPGEquationsFB& CPGSys, array_2D nodeParams)
{
    // Ensure that this hasn't already been assigned
    assert(m_nodeNumber == -1);
    
    m_pCPGSystem = &CPGSys;

    std::vector<double> params (11);
    params[0] = nodeParams[0][0]; // Frequency Offset
    params[1] = nodeParams[0][0]; // Frequency Scale
    params[2] = nodeParams[0][1]; // Radius Offset
    params[3] = nodeParams[0][1]; // Radius Scale
    params[4] = 1.0; // rConst (a constant)
    params[5] = 0.0; // dMin for descending commands
    params[6] = 5.0; // dMax for descending commands
    params[6] = 5.0; // dMax for descending commands
    params[7] = nodeParams[0][0]; // Omega (initialize variable)
    params[8] = nodeParams[0][2]; // Frequency feedback
    params[9] = nodeParams[0][3]; // Amplitude feedback
    params[10] = nodeParams[0][4]; // Phase feedback
    
    m_nodeNumber = CPGSys.addNode(params);
} 
