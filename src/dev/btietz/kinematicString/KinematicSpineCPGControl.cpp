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
 * @file KinematicSpineCPGControl.cpp
 * @brief A controller for the template class BaseSpineModelLearning
 * @author Brian Tietz
 * @version 1.0.0
 * $Id$
 */

#include "KinematicSpineCPGControl.h"

#include <string>


// Should include tgString, but compiler complains since its been
// included from BaseSpineModelLearning. Perhaps we should move things
// to a cpp over there
#include "core/tgSpringCableActuator.h"
#include "core/tgBasicActuator.h"
#include "controllers/tgImpedanceController.h"
#include "examples/learningSpines/tgCPGActuatorControl.h"
#include "dev/CPG_feedback/tgCPGCableControl.h"

#include "helpers/FileHelpers.h"

#include "learning/AnnealEvolution/AnnealEvolution.h"
#include "learning/Configuration/configuration.h"

#include "util/CPGEquations.h"
#include "util/CPGNode.h"

//#define LOGGING
#define USE_KINEMATIC

/**
 * Defining the adapters here assumes the controller is around and
 * attached for the lifecycle of the learning runs. I.E. that the setup
 * and teardown functions are used for tgModel
 */
KinematicSpineCPGControl::KinematicSpineCPGControl(BaseSpineCPGControl::Config config,	
												std::string args,
												std::string resourcePath,
                                                std::string ec,
                                                std::string nc) :
BaseSpineCPGControl(config, args, resourcePath, ec, nc)
{

    
}

void KinematicSpineCPGControl::setupCPGs(BaseSpineModelLearning& subject, array_2D nodeActions, array_4D edgeActions)
{
	    
    std::vector <tgSpringCableActuator*> allMuscles = subject.getAllMuscles();
    
    for (std::size_t i = 0; i < allMuscles.size(); i++)
    {
		#ifdef USE_KINEMATIC
            tgPIDController::Config config(5000.0, 0.0, 10.0, true); // Non backdrivable
            tgCPGCableControl* pStringControl = new tgCPGCableControl(config, m_config.controlTime);
        #else
            tgCPGStringControl* pStringControl = new tgCPGStringControl();
        #endif
        allMuscles[i]->attach(pStringControl);
        
        m_allControllers.push_back(pStringControl);
    }
    
    /// @todo: redo with for_each
    // First assign node numbers to the info Classes 
    for (std::size_t i = 0; i < m_allControllers.size(); i++)
    {
        m_allControllers[i]->assignNodeNumber(*m_pCPGSys, nodeActions);
    }
    
    // Then determine connectivity and setup string
    for (std::size_t i = 0; i < m_allControllers.size(); i++)
    {
        tgCPGActuatorControl * const pStringInfo = m_allControllers[i];
        assert(pStringInfo != NULL);
        pStringInfo->setConnectivity(m_allControllers, edgeActions);
        
        //String will own this pointer
        tgImpedanceController* p_ipc = new tgImpedanceController( m_config.tension,
                                                        m_config.kPosition,
                                                        m_config.kVelocity);
        if (m_config.useDefault)
        {
			pStringInfo->setupControl(*p_ipc);
		}
		else
		{
			pStringInfo->setupControl(*p_ipc, m_config.controlLength);
		}
    }
	
}
