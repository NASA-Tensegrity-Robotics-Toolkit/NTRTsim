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
 * @file LearningSpineJSON.cpp
 * @brief Controller for TetraSpineLearningModel
 * @author Brian Tietz
 * @date May 2014
 * @version 1.0.0
 * $Id$
 */

#include "LearningSpineJSON.h"

#include <string>


// Should include tgString, but compiler complains since its been
// included from TetraSpineLearningModel. Perhaps we should move things
// to a cpp over there
#include "core/tgSpringCableActuator.h"
#include "controllers/tgImpedanceController.h"

#include "learning/AnnealEvolution/AnnealEvolution.h"
#include "learning/Configuration/configuration.h"

#include "tgCPGStringControl_mod.h"

/**
 * Defining the adapters here assumes the controller is around and
 * attached for the lifecycle of the learning runs. I.E. that the setup
 * and teardown functions are used for tgModel
 */
LearningSpineJSON::LearningSpineJSON(BaseSpineCPGControl::Config config,
												std::string args,
                                                std::string ec,
                                                std::string nc) :
BaseSpineCPGControl(config, args)

{    
}

void LearningSpineJSON::setupCPGs(BaseSpineModelLearning& subject, array_2D nodeActions, array_4D edgeActions)
{
	std::vector <tgSpringCableActuator*> allMuscles = subject.getAllMuscles();
    
    for (std::size_t i = 0; i < allMuscles.size(); i++)
    {
		tgCPGStringControl_mod* pStringControl = new tgCPGStringControl_mod();
        allMuscles[i]->attach(pStringControl);
        std::cout << allMuscles[i]->getTags() << std::endl;
        m_allControllers.push_back(pStringControl);
    }
    
    assert(m_allControllers.size() == allMuscles.size());
    
    /// @todo: redo with for_each
    // First assign node numbers to the info Classes 
    for (std::size_t i = 0; i < m_allControllers.size(); i++)
    {
        m_allControllers[i]->assignNodeNumber(*m_pCPGSys, nodeActions);
    }

    double tension;
    double kPosition;
    double kVelocity;
    double controlLength;
    // Then determine connectivity and setup string
    for (std::size_t i = 0; i < m_allControllers.size(); i++)
    {
        tgCPGActuatorControl * const pStringInfo = m_allControllers[i];
        assert(pStringInfo != NULL);
        pStringInfo->setConnectivity(m_allControllers, edgeActions);
        
        if (allMuscles[i]->hasTag("inner top"))
        {
			tension = 2000.0;
            kPosition = 500.0;
            kVelocity = 100.0;
            controlLength = allMuscles[i]->getStartLength();
            //controlLength = 19.5;
		}
        else if (allMuscles[i]->hasTag("outer top"))
        {
            tension = 1000.0;
            kPosition = 500.0;
            kVelocity = 100.0;
            controlLength = 18.5;
        }
        else if (allMuscles[i]->hasTag("inner"))
        {
            tension = 1500.0;
            kPosition = 300.0;
            kVelocity = 100.0;
            controlLength = allMuscles[i]->getStartLength();
            //controlLength = 21.5;
        }
        else if (allMuscles[i]->hasTag("outer"))
        {
			tension = 800.0;
            kPosition = 300.0;
            kVelocity = 100.0;
            controlLength = 19.0 ;
		}
		else
		{
			throw std::runtime_error("Missing tags!");
		}

        tgImpedanceController* p_ipc = new tgImpedanceController( tension,
                                                        kPosition,
                                                        kVelocity);
        pStringInfo->setupControl(*p_ipc, controlLength);

    }
    
}
