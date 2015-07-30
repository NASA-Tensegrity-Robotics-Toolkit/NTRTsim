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
 * @file BaseSpineCPGControl.cpp
 * @brief A controller for the template class BaseSpineModelLearning
 * @author Brian Tietz
 * @version 1.0.0
 * $Id$
 */

#include "BaseSpineCPGControl.h"

#include <string>


// Should include tgString, but compiler complains since its been
// included from BaseSpineModelLearning. Perhaps we should move things
// to a cpp over there
#include "core/tgSpringCableActuator.h"
#include "controllers/tgImpedanceController.h"
#include "tgCPGActuatorControl.h"

#include "helpers/FileHelpers.h"

#include "learning/AnnealEvolution/AnnealEvolution.h"
#include "learning/Configuration/configuration.h"

#include "util/CPGEquations.h"
#include "util/CPGNode.h"

//#define LOGGING

using namespace std;

BaseSpineCPGControl::Config::Config(int ss,
										int tm,
										int om,
										int param,
										int segnum,
										double ct,
								        double la,
										double ha,
										double lp,
										double hp,
										double kt,
										double kp,
										double kv,
										bool def,
										double cl,
										double lf,
										double hf) :
	segmentSpan(ss),
	theirMuscles(tm),
	ourMuscles(om),
	params(param),
	segmentNumber(segnum),
	controlTime(ct),
	lowAmp(la),
	highAmp(ha),
	lowPhase(lp),
	highPhase(hp),
	tension(kt),
	kPosition(kp),
	kVelocity(kv),
	useDefault(def),
	controlLength(cl),
	lowFreq(lf),
	highFreq(hf)
{
    if (ss <= 0)
    {
        throw std::invalid_argument("segmentSpan parameter is negative.");
    }
    else if (tm <= 0)
    {
        throw std::invalid_argument("theirMuscles parameter is negative.");
    }
    else if (om <= 0)
    {
        throw std::invalid_argument("Our Muscles parameter is negative.");
    }
    else if (param <= 0)
    {
        throw std::invalid_argument("Edge parameters is negative.");
    }
    else if (segnum < 0)
    {
        throw std::invalid_argument("Segment number is negative.");
    }
    else if (ct < 0.0)
    {
        throw std::invalid_argument("control time is negative.");
    }
    else if (kt < 0.0)
    {
        throw std::invalid_argument("impedance control tension is negative.");
    }
    else if (kp < 0.0)
    {
        throw std::invalid_argument("impedance control position is negative.");
    }
    else if (kv < 0.0)
    {
        throw std::invalid_argument("impedance control velocity is negative.");
    }
    else if (cl < 0.0)
    {
        throw std::invalid_argument("Control Length is negative.");
    }
}

/**
 * Defining the adapters here assumes the controller is around and
 * attached for the lifecycle of the learning runs. I.E. that the setup
 * and teardown functions are used for tgModel
 */
BaseSpineCPGControl::BaseSpineCPGControl(BaseSpineCPGControl::Config config,	
												std::string args,
												std::string resourcePath,
                                                std::string ec,
                                                std::string nc) :
m_config(config),
edgeConfigFilename(ec),
nodeConfigFilename(nc),
// Evolution assumes no pre-processing was done on these names
edgeEvolution(args + "_edge", ec, resourcePath),
// Can't have identical args or they'll overwrite each other
nodeEvolution(args + "_node", nc, resourcePath),
// Will be overwritten by configuration data
nodeLearning(false),
edgeLearning(false),
m_dataObserver("logs/TCData"),
m_pCPGSys(NULL),
m_updateTime(0.0),
bogus(false)
{
	std::string path;
	if (resourcePath != "")
	{
		path = FileHelpers::getResourcePath(resourcePath);
	}
	else
	{
		path = "";
	}
	
    nodeConfigData.readFile(path + nodeConfigFilename);
    edgeConfigData.readFile(path + edgeConfigFilename);
    nodeLearning = nodeConfigData.getintvalue("learning");
    edgeLearning = edgeConfigData.getintvalue("learning");
    
}

BaseSpineCPGControl::~BaseSpineCPGControl() 
{
    scores.clear();
}

void BaseSpineCPGControl::onSetup(BaseSpineModelLearning& subject)
{
    // Maximum number of sub-steps allowed by CPG
	m_pCPGSys = new CPGEquations(200);
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
    
    array_4D edgeParams = scaleEdgeActions(edgeAdapter.step(dt, state));
    array_2D nodeParams = scaleNodeActions(nodeAdapter.step(dt, state));
    
    setupCPGs(subject, nodeParams, edgeParams);
    
    initConditions = subject.getSegmentCOM(m_config.segmentNumber);
#ifdef LOGGING // Conditional compile for data logging    
    m_dataObserver.onSetup(subject);
#endif    
    
#if (0) // Conditional Compile for debug info
    std::cout << *m_pCPGSys << std::endl;
#endif    
    m_updateTime = 0.0;
    bogus = false;
}

void BaseSpineCPGControl::setupCPGs(BaseSpineModelLearning& subject, array_2D nodeActions, array_4D edgeActions)
{
	    
    std::vector <tgSpringCableActuator*> allMuscles = subject.getAllMuscles();
    
    for (std::size_t i = 0; i < allMuscles.size(); i++)
    {
		tgCPGActuatorControl* pStringControl = new tgCPGActuatorControl();
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

void BaseSpineCPGControl::onStep(BaseSpineModelLearning& subject, double dt)
{
    m_updateTime += dt;
    if (m_updateTime >= m_config.controlTime)
    {
        std::size_t numControllers = subject.getNumberofMuslces();
        
        double descendingCommand = 2.0;
        std::vector<double> desComs (numControllers, descendingCommand);
        
        m_pCPGSys->update(desComs, m_updateTime);
#ifdef LOGGING // Conditional compile for data logging        
        m_dataObserver.onStep(subject, m_updateTime);
#endif
		notifyStep(m_updateTime);
        m_updateTime = 0;
    }
    
    double currentHeight = subject.getSegmentCOM(m_config.segmentNumber)[1];
    
    /// @todo add to config
    if (currentHeight > 25 || currentHeight < 1.0)
    {
		/// @todo if bogus, stop trial (reset simulation)
		bogus = true;
	}
}

void BaseSpineCPGControl::onTeardown(BaseSpineModelLearning& subject)
{
    scores.clear();
    // @todo - check to make sure we ran for the right amount of time
    
    std::vector<double> finalConditions = subject.getSegmentCOM(m_config.segmentNumber);
    
    const double newX = finalConditions[0];
    const double newZ = finalConditions[2];
    const double oldX = initConditions[0];
    const double oldZ = initConditions[2];
    
    const double distanceMoved = sqrt((newX-oldX) * (newX-oldX) + 
                                        (newZ-oldZ) * (newZ-oldZ));
    
    if (bogus)
    {
		scores.push_back(-1.0);
    }
    else
    {
		scores.push_back(distanceMoved);
	}
    
    /// @todo - consolidate with other controller classes. 
    /// @todo - return length scale as a parameter
    double totalEnergySpent=0;
    
    std::vector<tgSpringCableActuator* > tmpStrings = subject.getAllMuscles();
    
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
    
    delete m_pCPGSys;
    m_pCPGSys = NULL;
    
    for(size_t i = 0; i < m_allControllers.size(); i++)
    {
		delete m_allControllers[i];
	}
	m_allControllers.clear();
}

const double BaseSpineCPGControl::getCPGValue(std::size_t i) const
{
	// Error handling on input done in CPG_Equations
	return (*m_pCPGSys)[i];
}

double BaseSpineCPGControl::getScore() const
{
	if (scores.size() == 2)
	{
		return scores[0];
	}
	else
	{
		throw std::runtime_error("Called before scores were obtained!");
	}
}
	

array_4D BaseSpineCPGControl::scaleEdgeActions  
                            (vector< vector <double> > actions)
{
    std::size_t numControllers = edgeConfigData.getintvalue("numberOfControllers");
    
    // Ensure reading from the same file
    assert(numControllers == actions.size());
    assert(actions[0].size() == 2);
    
    double lowerLimit = m_config.lowPhase;
    double upperLimit = m_config.highPhase;
    double range = upperLimit - lowerLimit;
    
    array_4D actionList(boost::extents[m_config.segmentSpan]
										[m_config.theirMuscles]
										[m_config.ourMuscles]
										[m_config.params]);
    
    /* Horrid while loop to populate upper diagonal of matrix, since
    * its symmetric and we want to minimze parameters used in learing
    * note that i==1, j==k will refer to the same muscle
    * @todo use boost to set up array so storage is only allocated for 
    * elements that are used
    */
    int i = 0;
    int j = 0;
    int k = 0;

    while (i < m_config.segmentSpan)
    {
        while(j < m_config.theirMuscles)
        {
            while(k < m_config.ourMuscles)
            {
                if (actions.empty())
                {
                    std::cout << "ran out before table populated!"
                    << std::endl;
                    break;
                }
                else
                {
                    if (i == 1 && j == k)
                    {
                        // std::cout << "Skipped identical muscle" << std::endl;
                        //Skip since its the same muscle
                    }
                    else
                    {
                        std::vector<double> edgeParam = actions.back();
                        // Weight from 0 to 1
                        actionList[i][j][k][0] = edgeParam[0];
                        // Phase offset from -pi to pi
                        actionList[i][j][k][1] = edgeParam[1] * 
                                                (range) + lowerLimit;
                        actions.pop_back();
                    }
                }
                k++;
            }
            j++;
            k = j;
            
        }
        j = 0;
        k = 0;
        i++;
    }
    
    assert(actions.empty());
    
    return actionList;
}
array_2D BaseSpineCPGControl::scaleNodeActions  
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
    
	limits[0][0] = m_config.lowFreq;
	limits[1][0] = m_config.highFreq;
	limits[0][1] = m_config.lowAmp;
	limits[1][1] = m_config.highAmp;
    
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
