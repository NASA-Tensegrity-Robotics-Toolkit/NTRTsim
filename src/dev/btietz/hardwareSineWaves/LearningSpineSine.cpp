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

#include "LearningSpineSine.h"

#include <string>


// Should include tgString, but compiler complains since its been
// included from TetraSpineLearningModel. Perhaps we should move things
// to a cpp over there
#include "core/tgSpringCableActuator.h"
#include "core/tgBasicActuator.h"
#include "controllers/tgImpedanceController.h"

#include "learning/AnnealEvolution/AnnealEvolution.h"
#include "learning/Configuration/configuration.h"

#include "tgSineStringControl.h"

using namespace std;

/**
 * Defining the adapters here assumes the controller is around and
 * attached for the lifecycle of the learning runs. I.E. that the setup
 * and teardown functions are used for tgModel
 */
LearningSpineSine::LearningSpineSine(BaseSpineCPGControl::Config config,
												std::string args,
                                                std::string ec,
                                                std::string nc) :
BaseSpineCPGControl(config, args)

{    
}

void LearningSpineSine::onSetup(BaseSpineModelLearning& subject)
{
    //Initialize the Learning Adapters
    nodeAdapter.initialize(&nodeEvolution,
                            nodeLearning,
                            nodeConfigData);
    edgeAdapter.initialize(&edgeEvolution,
                            edgeLearning,
                            edgeConfigData);
    /* Empty vector signifying no state information
     * All parameters are stateless parameters, so we can get away with
     * only doing this once
     */
    std::vector<double> state;
    double dt = 0;
    
    array_2D edgeParams = scalePhaseActions(edgeAdapter.step(dt, state));
    array_2D nodeParams = scaleNodeActions(nodeAdapter.step(dt, state));
    
    setupWaves(subject, nodeParams, edgeParams);
    
    initConditions = subject.getSegmentCOM(m_config.segmentNumber);
#if (0) // Conditional compile for data logging    
    m_dataObserver.onSetup(subject);
#endif    
    
  
    m_updateTime = 0.0;
}

void LearningSpineSine::onStep(BaseSpineModelLearning& subject, double dt)
{
	/// Basically nothing to do. Sine controllers will take care of themselves
    m_updateTime += dt;
    if (m_updateTime >= m_config.controlTime)
    {

#if (0) // Conditional compile for data logging        
        m_dataObserver.onStep(subject, m_updateTime);
#endif
		notifyStep(m_updateTime);
        m_updateTime = 0;
    }
}

void LearningSpineSine::onTeardown(BaseSpineModelLearning& subject)
{
    std::vector<double> scores;
    // @todo - check to make sure we ran for the right amount of time
    
    std::vector<double> finalConditions = subject.getSegmentCOM(m_config.segmentNumber);
    
    const double newX = finalConditions[0];
    const double newZ = finalConditions[2];
    const double oldX = initConditions[0];
    const double oldZ = initConditions[2];
    
    const double distanceMoved = sqrt((newX-oldX) * (newX-oldX) + 
                                        (newZ-oldZ) * (newZ-oldZ));
    
    scores.push_back(distanceMoved);
    
    /// @todo - consolidate with other controller classes. 
    /// @todo - return length scale as a parameter
    double totalEnergySpent=0;
    
    vector<tgSpringCableActuator* > tmpSCAs = subject.getAllMuscles();
    vector<tgBasicActuator* > tmpStrings = tgCast::filter<tgSpringCableActuator, tgBasicActuator>(tmpSCAs);
    for(int i=0; i<tmpStrings.size(); i++)
    {
        tgSpringCableActuator::SpringCableActuatorHistory stringHist = tmpStrings[i]->getHistory();
        
        for(int j=1; j<stringHist.tensionHistory.size(); j++)
        {
            const double previousTension = stringHist.tensionHistory[j-1];
            const double previousLength = stringHist.restLengths[j-1];
            const double currentLength = stringHist.restLengths[j];
            //TODO: examine this assumption - free spinning motor may require more power
            double motorSpeed = (currentLength-previousLength);
            if(motorSpeed > 0) // Vestigial code
                motorSpeed = 0;
            const double workDone = previousTension * motorSpeed;
            totalEnergySpent += workDone;
        }
    }
    
    scores.push_back(totalEnergySpent);
    
    edgeAdapter.endEpisode(scores);
    nodeAdapter.endEpisode(scores);

    for(size_t i = 0; i < m_sineControllers.size(); i++)
    {
		delete m_sineControllers[i];
	}
	m_sineControllers.clear();
	m_allControllers.clear();
}

void LearningSpineSine::setupWaves(BaseSpineModelLearning& subject, array_2D nodeActions, array_2D edgeActions)
{
	std::vector <tgSpringCableActuator*> allMuscles = subject.getAllMuscles();
    
    double tension;
    double kPosition;
    double kVelocity;
    double controlLength;
    
    for (std::size_t i = 0; i < allMuscles.size(); i++)
    {
		if (allMuscles[i]->hasTag("inner top"))
        {
			tension = 2000.0;
            kPosition = 500.0;
            kVelocity = 100.0;
            controlLength = allMuscles[i]->getStartLength();
		}
        else if (allMuscles[i]->hasTag("outer top"))
        {
            tension = 1000.0;
            kPosition = 500.0;
            kVelocity = 100.0;
            controlLength = 19.5;
        }
        else if (allMuscles[i]->hasTag("inner"))
        {
            tension = 1500.0;
            kPosition = 100.0;
            kVelocity = 100.0;
            controlLength = allMuscles[i]->getStartLength();
        }
        else if (allMuscles[i]->hasTag("outer"))
        {
			tension = 800.0;
            kPosition = 100.0;
            kVelocity = 100.0;
            controlLength = 19.5 ;
		}
		else
		{
			throw std::runtime_error("Missing tags!");
		}

        tgImpedanceController* p_ipc = new tgImpedanceController( tension,
                                                        kPosition,
                                                        kVelocity);
        
        tgSineStringControl* pStringControl = new tgSineStringControl(m_config.controlTime,
																		p_ipc,
																		nodeActions[0][0],
																		nodeActions[0][1],
																		edgeActions[i][0],
																		0.0, // Repeat learning this too? Unlikely to be helpful
																		controlLength);

		
		allMuscles[i]->attach(pStringControl);
        m_sineControllers.push_back(pStringControl);
    }
    
    assert(m_sineControllers.size() == allMuscles.size());
  


    
}

array_2D LearningSpineSine::scalePhaseActions  
                            (vector< vector <double> > actions)
{
    std::size_t numControllers = edgeConfigData.getintvalue("numberOfControllers");
    std::size_t numActions = edgeConfigData.getintvalue("numberOfActions");
    
    assert( actions.size() == numControllers);
    assert( actions[0].size() == numActions);
    
    array_2D edgeActions(boost::extents[numControllers][numActions]);
    
    array_2D limits(boost::extents[2][numActions]);
    
    // Check if we need to update limits
    assert(numActions == 1);
 
    
	limits[0][0] = m_config.lowPhase;
	limits[1][0] = m_config.highPhase;
    
    // This one is square
    for( std::size_t i = 0; i < numControllers; i++)
    {
        for( std::size_t j = 0; j < numActions; j++)
        {
            edgeActions[i][j] = ( actions[i][j] *  
                    (limits[1][j] - limits[0][j])) + limits[0][j];
        }
    }
    
    return edgeActions;
}

array_2D LearningSpineSine::scaleNodeActions  
                            (vector< vector <double> > actions)
{
    std::size_t numControllers = nodeConfigData.getintvalue("numberOfControllers");
    std::size_t numActions = nodeConfigData.getintvalue("numberOfActions");
    
    assert( actions.size() == numControllers);
    assert( actions[0].size() == numActions);
    
    array_2D nodeActions(boost::extents[numControllers][numActions]);
    
    array_2D limits(boost::extents[2][numActions]);
    
    // Check if we need to update limits
    assert(numActions == 2);
    
    
	limits[0][0] = m_config.lowAmp;
	limits[1][0] = m_config.highAmp;
	limits[1][1] = m_config.lowFreq;
	limits[1][1] = m_config.highFreq;
    
    // This one is square
    for( std::size_t i = 0; i < numControllers; i++)
    {
        for( std::size_t j = 0; j < numActions; j++)
        {
            nodeActions[i][j] = ( actions[i][j] *  
                    (limits[1][j] - limits[0][j])) + limits[0][j];
        }
    }
    
    return nodeActions;
}
