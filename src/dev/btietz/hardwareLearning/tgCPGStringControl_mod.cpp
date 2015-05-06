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

#include "tgCPGStringControl_mod.h"

#include "controllers/tgImpedanceController.h"
#include "util/CPGEquations.h"
#include "core/tgBasicActuator.h"

#include <iostream>
#include <stdexcept>

tgCPGStringControl_mod::tgCPGStringControl_mod(const double controlStep) :
tgCPGActuatorControl(controlStep)
{
    if (m_controlStep < 0.0)
    {
        throw std::invalid_argument("Negative control step");
    }
}

tgCPGStringControl_mod::~tgCPGStringControl_mod()
{

}

void tgCPGStringControl_mod::onStep(tgSpringCableActuator& subject, double dt)
{
    m_controlTime += dt;
	m_totalTime += dt;
    
    tgBasicActuator& m_sca = *(tgCast::cast<tgSpringCableActuator, tgBasicActuator>(subject));
    
    /// @todo this fails if its attached to multiple controllers!
    /// is there a way to track _global_ time at this level
    if (m_controlTime >= m_controlStep)
    {
		// Encoder inversion for hardware comparison.
		if (m_nodeNumber == 2 || m_nodeNumber == 4 || m_nodeNumber == 5 || m_nodeNumber == 6 || m_nodeNumber == 7 || m_nodeNumber == 8)//||m_nodeNumber == 9|| m_nodeNumber == 10 )
		{
			m_commandedTension = motorControl().control(m_sca, m_controlTime, controlLength(), -getCPGValue());
		}
		else
		{
			m_commandedTension = motorControl().control(m_sca, m_controlTime, controlLength(), getCPGValue());
		}
        m_controlTime = 0;
    }
    else
    {
		m_sca.moveMotors(dt);
	}
}
