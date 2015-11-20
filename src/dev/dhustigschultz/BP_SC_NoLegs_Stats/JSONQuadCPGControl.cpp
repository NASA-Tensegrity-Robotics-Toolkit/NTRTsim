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
 * @file JSONCPGControl.cpp
 * @brief A controller for the template class BaseSpineModelLearning
 * @author Brian Mirletz
 * @version 1.1.0
 * $Id$
 */

#include "JSONQuadCPGControl.h"

#include <string>


// Should include tgString, but compiler complains since its been
// included from BaseSpineModelLearning. Perhaps we should move things
// to a cpp over there
#include "core/tgSpringCableActuator.h"
#include "controllers/tgImpedanceController.h"
#include "examples/learningSpines/tgCPGActuatorControl.h"
#include "dev/CPG_feedback/tgCPGCableControl.h"
#include "examples/learningSpines/BaseSpineCPGControl.h"

#include "helpers/FileHelpers.h"

#include "util/CPGEquations.h"
#include "util/CPGNode.h"

// JSON
#include <json/json.h>

#include <exception>

//#define LOGGING

using namespace std;

JSONQuadCPGControl::Config::Config(int ss,
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
    lowFreq(lf),
    highFreq(hf),
	lowPhase(lp),
	highPhase(hp),
	tension(kt),
	kPosition(kp),
	kVelocity(kv),
	useDefault(def),
	controlLength(cl)
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
JSONQuadCPGControl::JSONQuadCPGControl(JSONQuadCPGControl::Config config,	
												std::string args,
												std::string resourcePath) :
m_pCPGSys(NULL),
m_config(config),
m_dataObserver("logs/TCData"),
m_updateTime(0.0),
bogus(false)
{
	if (resourcePath != "")
	{
		controlFilePath = FileHelpers::getResourcePath(resourcePath);
	}
	else
	{
		controlFilePath = "";
	}
    
    controlFilename = controlFilePath + args;
}

JSONQuadCPGControl::~JSONQuadCPGControl() 
{
    scores.clear();
}

void JSONQuadCPGControl::onSetup(BaseQuadModelLearning& subject)
{
    // Maximum number of sub-steps allowed by CPG
	m_pCPGSys = new CPGEquations(200);
    //Initialize the Learning Adapters

    Json::Value root; // will contains the root value after parsing.
    Json::Reader reader;

    bool parsingSuccessful = reader.parse( FileHelpers::getFileString(controlFilename.c_str()), root );
    if ( !parsingSuccessful )
    {
        // report to the user the failure and their locations in the document.
        std::cout << "Failed to parse configuration\n"
            << reader.getFormattedErrorMessages();
        throw std::invalid_argument("Bad filename for JSON");
    }
    // Get the value of the member of root named 'encoding', return 'UTF-8' if there is no
    // such member.
    Json::Value nodeVals = root.get("nodeVals", "UTF-8");
    Json::Value edgeVals = root.get("edgeVals", "UTF-8");
    
    nodeVals = nodeVals.get("params", "UTF-8");
    edgeVals = edgeVals.get("params", "UTF-8");
    
    array_4D edgeParams = scaleEdgeActions(edgeVals);
    array_2D nodeParams = scaleNodeActions(nodeVals);
    
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

void JSONQuadCPGControl::setupCPGs(BaseQuadModelLearning& subject, array_2D nodeActions, array_4D edgeActions)
{
	    
    std::vector <tgSpringCableActuator*> allMuscles = subject.getAllMuscles();
    
    for (std::size_t i = 0; i < allMuscles.size(); i++)
    {
#if (1)
        tgPIDController::Config config(20000.0, 0.0, 5.0, true); // Non backdrivable
        tgCPGCableControl* pStringControl = new tgCPGCableControl(config);
#else
		tgCPGActuatorControl* pStringControl = new tgCPGActuatorControl();
#endif // Update for kinematic cables
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

void JSONQuadCPGControl::onStep(BaseQuadModelLearning& subject, double dt)
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

void JSONQuadCPGControl::onTeardown(BaseQuadModelLearning& subject)
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
    
    for(std::size_t  i=0; i<tmpStrings.size(); i++)
    {
        tgSpringCableActuator::SpringCableActuatorHistory stringHist = tmpStrings[i]->getHistory();
        
        for(std::size_t j=1; j<stringHist.tensionHistory.size(); j++)
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
    
        std::cout << "Dist travelled " << scores[0] << std::endl;
    
    Json::Value root; // will contains the root value after parsing.
    Json::Reader reader;

    bool parsingSuccessful = reader.parse( FileHelpers::getFileString(controlFilename.c_str()), root );
    if ( !parsingSuccessful )
    {
        // report to the user the failure and their locations in the document.
        std::cout << "Failed to parse configuration\n"
            << reader.getFormattedErrorMessages();
        throw std::invalid_argument("Bad filename for JSON");
    }
    
    Json::Value prevScores = root.get("scores", Json::nullValue);
    
    Json::Value subScores;
    subScores["distance"] = scores[0];
    subScores["energy"] = totalEnergySpent;
    
    prevScores.append(subScores);
    root["scores"] = prevScores;
    
    ofstream payloadLog;
    payloadLog.open(controlFilename.c_str(),ofstream::out);
    
    payloadLog << root << std::endl;
    
    delete m_pCPGSys;
    m_pCPGSys = NULL;
    
    for(size_t i = 0; i < m_allControllers.size(); i++)
    {
		delete m_allControllers[i];
	}
	m_allControllers.clear();
}

const double JSONQuadCPGControl::getCPGValue(std::size_t i) const
{
	// Error handling on input done in CPG_Equations
	return (*m_pCPGSys)[i];
}

double JSONQuadCPGControl::getScore() const
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
	

array_4D JSONQuadCPGControl::scaleEdgeActions  
                            (Json::Value edgeParam)
{
    assert(edgeParam[0].size() == 2);
    
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
    
    // Quirk of the old learning code. Future examples can move forward
    Json::Value::iterator edgeIt = edgeParam.end();
    
    int count = 0;
    
    while (i < m_config.segmentSpan)
    {
        while(j < m_config.theirMuscles)
        {
            while(k < m_config.ourMuscles)
            {
                if (edgeIt == edgeParam.begin())
                {
                    std::cout << "ran out before table populated!"
                    << std::endl;
                    /// @todo consider adding exception here
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
                        edgeIt--;
                        Json::Value edgeParam = *edgeIt;
                        assert(edgeParam.size() == 2);
                        // Weight from 0 to 1
                        actionList[i][j][k][0] = edgeParam[0].asDouble();
                        //std::cout << actionList[i][j][k][0] << " ";
                        // Phase offset from -pi to pi
                        actionList[i][j][k][1] = edgeParam[1].asDouble() * 
                                                (range) + lowerLimit;
                        //std::cout <<  actionList[i][j][k][1] << std::endl;
                        count++;
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
    
    std::cout<< "Params used: " << count << std::endl;
    
    assert(edgeParam.begin() == edgeIt);
    
    return actionList;
}
array_2D JSONQuadCPGControl::scaleNodeActions  
                            (Json::Value actions)
{
    std::size_t numControllers = actions.size();
    std::size_t numActions = actions[0].size();
    
    array_2D nodeActions(boost::extents[numControllers][numActions]);
    
    array_2D limits(boost::extents[2][numActions]);
    
    // Check if we need to update limits
    assert(numActions == 2);
    
	limits[0][0] = m_config.lowFreq;
	limits[1][0] = m_config.highFreq;
	limits[0][1] = m_config.lowAmp;
	limits[1][1] = m_config.highAmp;
    
    Json::Value::iterator nodeIt = actions.begin();
    
    // This one is square
    for( std::size_t i = 0; i < numControllers; i++)
    {
        Json::Value nodeParam = *nodeIt;
        for( std::size_t j = 0; j < numActions; j++)
        {
            nodeActions[i][j] = ( (nodeParam.get(j, 0.0)).asDouble() *  
                    (limits[1][j] - limits[0][j])) + limits[0][j];
        }
        nodeIt++;
    }
    
    return nodeActions;
}
