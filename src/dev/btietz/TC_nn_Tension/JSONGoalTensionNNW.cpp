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
 * @file JSONGoalTensionNNW.cpp
 * @brief A controller for the template class BaseSpineModelLearning
 * @author Brian Mirletz
 * @version 1.1.0
 * $Id$
 */

#include "JSONGoalTensionNNW.h"


// Should include tgString, but compiler complains since its been
// included from BaseSpineModelLearning. Perhaps we should move things
// to a cpp over there
#include "core/tgSpringCableActuator.h"
#include "core/tgBasicActuator.h"
#include "controllers/tgImpedanceController.h"
#include "examples/learningSpines/tgCPGActuatorControl.h"
#include "dev/CPG_feedback/tgCPGCableControl.h"
#include "dev/btietz/kinematicString/tgSCASineControl.h"

#include "examples/learningSpines/BaseSpineModelLearning.h"
#include "dev/btietz/TC_goal/BaseSpineModelGoal.h"
#include "helpers/FileHelpers.h"

#include "dev/CPG_feedback/CPGEquationsFB.h"
#include "dev/CPG_feedback/CPGNodeFB.h"

#include "neuralNet/Neural Network v2/neuralNetwork.h"

#include <json/json.h>

#include <string>
#include <iostream>
#include <vector>

//#define LOGGING
#define USE_KINEMATIC

using namespace std;

/**
 * Defining the adapters here assumes the controller is around and
 * attached for the lifecycle of the learning runs. I.E. that the setup
 * and teardown functions are used for tgModel
 */
JSONGoalTensionNNW::JSONGoalTensionNNW(JSONGoalControl::Config config,	
                                                std::string args,
                                                std::string resourcePath) :
JSONGoalControl(config, args, resourcePath)
{
    // Path and filename handled by base class
    
}

JSONGoalTensionNNW::~JSONGoalTensionNNW()
{
}

void JSONGoalTensionNNW::onSetup(BaseSpineModelLearning& subject)
{
    m_totalTime = 0;
	m_pCPGSys = new CPGEquationsFB(200);

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
    
    Json::Value feedbackParams = root.get("feedbackVals", "UTF-8");
    feedbackParams = feedbackParams.get("params", "UTF-8");
    
    // Setup neural network
    m_config.numStates = feedbackParams.get("numStates", "UTF-8").asInt();
    m_config.numActions = feedbackParams.get("numActions", "UTF-8").asInt();
    m_config.numHidden = feedbackParams.get("numHidden", "UTF-8").asInt();
    
    std::string nnFile = controlFilePath + feedbackParams.get("neuralFilename", "UTF-8").asString();
    
    nn = new neuralNetwork(m_config.numStates, m_config.numHidden, m_config.numActions);
    
    nn->loadWeights(nnFile.c_str());
    
    
    Json::Value goalParams = root.get("goalVals", "UTF-8");
    goalParams = goalParams.get("params", "UTF-8");
    
    // Setup neural network
    m_config.goalStates = goalParams.get("numStates", "UTF-8").asInt();
    m_config.goalActions = goalParams.get("numActions", "UTF-8").asInt();
    m_config.goalHidden = goalParams.get("numHidden", "UTF-8").asInt();
    
    std::string nnFile_goal = controlFilePath + goalParams.get("neuralFilename", "UTF-8").asString();
    
    nn_goal = new neuralNetwork(m_config.goalStates, m_config.goalHidden, m_config.goalActions);
    
    nn_goal->loadWeights(nnFile_goal.c_str());
    
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

void JSONGoalTensionNNW::onStep(BaseSpineModelLearning& subject, double dt)
{
    m_updateTime += dt;
    m_totalTime += dt;
    double currentHeight = subject.getSegmentCOM(m_config.segmentNumber)[1];
    
    if (m_updateTime >= m_config.controlTime)
    {
        
        
#if (1) // Goal and cable

    std::vector<double> desComs = getFeedback(subject);
    
    const BaseSpineModelGoal* goalSubject = tgCast::cast<BaseSpineModelLearning, BaseSpineModelGoal>(subject);
    
    getGoalFeedback(goalSubject);
    
#else // Just goal
    std::size_t numControllers = subject.getNumberofMuslces() * 3;
    
    double descendingCommand = 0.0;
    std::vector<double> desComs (numControllers, descendingCommand);
    
    const BaseSpineModelGoal* goalSubject = tgCast::cast<BaseSpineModelLearning, BaseSpineModelGoal>(subject);
        
    getGoalFeedback(goalSubject);
    
#endif       
        try
        {
            m_pCPGSys->update(desComs, m_updateTime);
        }
        catch (std::runtime_error& e)
        {
            //  Stops the trial immediately,  lets teardown know it broke
            bogus = true;
            throw (e);
        }
        
#ifdef LOGGING // Conditional compile for data logging        
        m_dataObserver.onStep(subject, m_updateTime);
#endif
		notifyStep(m_updateTime);
        m_updateTime = 0;
        //std::cout << m_totalTime << " " << currentHeight<< std::endl;
    }
    
    
#if (0)

    /// @todo add to config
    if (currentHeight > 25 || currentHeight < 1.0)
    {
		/// @todo if bogus, stop trial (reset simulation)
		bogus = true;
		throw std::runtime_error("Height out of range");
	}
#endif
}

void JSONGoalTensionNNW::onTeardown(BaseSpineModelLearning& subject)
{
    scores.clear();
    // @todo - check to make sure we ran for the right amount of time
    
    std::vector<double> finalConditions = subject.getSegmentCOM(m_config.segmentNumber);
    
    const double newX = finalConditions[0];
    const double newZ = finalConditions[2];
    const double oldX = initConditions[0];
    const double oldZ = initConditions[2];
    
    const BaseSpineModelGoal* goalSubject = tgCast::cast<BaseSpineModelLearning, BaseSpineModelGoal>(subject);
    
    const double totalDistanceMoved = sqrt((newX-oldX) * (newX-oldX) + 
                                    (newZ-oldZ) * (newZ-oldZ));
    
    const double distanceMoved = calculateDistanceMoved(goalSubject);
    
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
    
    for(std::size_t i=0; i<tmpStrings.size(); i++)
    {
        tgSpringCableActuator::SpringCableActuatorHistory stringHist = tmpStrings[i]->getHistory();
        
        std::size_t histSize = stringHist.tensionHistory.size();
        
        double dt = m_totalTime / (double)histSize;
        
        //std::cout << "Estimated dt: " << dt << std::endl;
    #if (0)
    
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
        
    #else
        for(std::size_t j=0; j < histSize; j++)
        {
            const double previousTension = stringHist.tensionHistory[j];
            double motorSpeed = stringHist.lastVelocities[j];
            // Integrating power over time
            const double workDone = previousTension * motorSpeed * dt;
            totalEnergySpent += workDone;
        }
    #endif
    }
    
    scores.push_back(totalEnergySpent);
    
    std::cout << "Dist travelled towards goal " << scores[0] << " Total Distance Travelled " << totalDistanceMoved ;
    std::cout << " Energy Spent: " << scores[1] << std::endl;
    
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
    subScores["distance"] = distanceMoved;
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
    
    delete nn_goal;
}

std::vector<double> JSONGoalTensionNNW::getGoalFeedback(const BaseSpineModelGoal* subject)
{
    const int nSeg = subject->getSegments() - 1;
    
    // Only one segment has no actuators, and means we can't get heading.
    assert(nSeg > 0);
    
    // Get heading and generate feedback vector
    const btVector3 currentPosVector = subject->getSegmentCOMVector(m_config.segmentNumber);
    
    const btVector3 goalPosition = subject->goalBoxPosition();
    
    btVector3 desiredHeading = (goalPosition - currentPosVector).normalize();
    
    // Get current orientation
    /// @TODO should this be configurable?
    const btVector3 firstSegment = subject->getSegmentCOMVector(0);
    const btVector3 secondSegment = subject->getSegmentCOMVector(1);
    
    btVector3 currentHeading = (firstSegment - secondSegment).normalize();
    
    std::vector<double> state;
    state.push_back(desiredHeading.getX());
    state.push_back(desiredHeading.getZ());
    state.push_back(currentHeading.getX());
    state.push_back(currentHeading.getZ());
    
    double *inputs = new double[m_config.goalStates];
    for (std::size_t i = 0; i < state.size(); i++)
    {
        assert(state[i] >= -1.0 && state[i] <= 1.0);
        // Don't scale! Sigmoid can handle this range
        inputs[i]=state[i];
    }
        
    double *output = nn_goal->feedForwardPattern(inputs);
    
    vector<double> actions;
    
#if (0)    
    for(int j=0;j<m_config.goalActions;j++)
    {
        std::cout << output[j] << " ";
    }
    std::cout << std::endl;
#endif
    
    for(int j=0;j<m_config.goalActions;j++)
    {
        actions.push_back(output[j]);
    }
    
    transformFeedbackActions(actions);
    
    assert(m_config.goalActions * nSeg == m_allControllers.size());
    
    // Duplicate the actions across segments
    for (int i = 0; i != nSeg; i++)
    {
        for(int j=0;j<m_config.goalActions;j++)
        {
            m_allControllers[i * m_config.goalActions + j]->updateTensionSetpoint(actions[j] * m_config.tensFeedback + m_config.tensFeedback);
        }
    }

    
    
    return actions;
}

