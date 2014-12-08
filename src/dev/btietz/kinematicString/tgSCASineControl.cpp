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

#include "tgSCASineControl.h"

#include "core/Muscle2P.h"
#include "tgImpedanceController.h"

#include <iostream>
#include <stdexcept>

tgSineStringControl::tgSineStringControl(const double controlStep,
											tgImpedanceController* p_ipc,
											tgPIDController::Config pidConfig,
											const double amplitude,
											const double frequency,
											const double phase,
											const double offset,
											const double length) :
m_controlTime(0.0),
m_totalTime(0.0),
m_controlStep(controlStep),
m_commandedTension(0.0),
cpgAmplitude(amplitude),
cpgFrequency(frequency),
phaseOffset(phase),
offsetSpeed(offset),
cycle(0.0),
target (0.0),
m_controlLength(length),
m_tempConfig(pidConfig),
m_PIDController(NULL),
m_pMotorControl(p_ipc)
{
    if (m_controlStep < 0.0)
    {
        throw std::invalid_argument("Negative control step");
    }
    
    /// @todo check all of the above.
    
    assert(p_ipc);
}

tgSineStringControl::~tgSineStringControl()
{

}

void tgSineStringControl::onAttach(tgKinematicString& subject)
{
	m_PIDController = new tgPIDController(&subject, m_tempConfig);
}

void tgSineStringControl::onStep(tgKinematicString& subject, double dt)
{
	assert(&subject == m_PIDController->getControllable());
	
    m_controlTime += dt;
	m_totalTime += dt;
    /// @todo this fails if its attached to multiple controllers!
    /// is there a way to track _global_ time at this level
    if (m_controlTime >= m_controlStep)
    {
		// Yep, its a misnomer. Had to change it for In Won
		cycle = cos(m_totalTime  * 2.0 * M_PI * cpgFrequency + phaseOffset);
        target = cycle*cpgAmplitude + offsetSpeed;
	#if (0)	
		if (phaseOffset == 0.0 && m_totalTime < 4.0)
		{
			target = 0.0;
		}
	#endif
	    // dt is just passed through to PID controller	
		m_commandedTension = m_pMotorControl->control(*m_PIDController, dt, m_controlLength, target);
		//std::cout << m_commandedTension << std::endl;
        m_controlTime = 0;
    }
    else
    {
		const double currentTension = subject.getTension();
		m_PIDController->control(dt, m_commandedTension, currentTension);
	}

}
