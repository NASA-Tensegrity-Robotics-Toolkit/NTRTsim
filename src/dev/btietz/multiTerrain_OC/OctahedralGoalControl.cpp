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
 * @file JSONGoalControl.cpp
 * @brief A controller for the template class BaseSpineModelLearning
 * @author Brian Mirletz
 * @version 1.1.0
 * $Id$
 */

#include "OctahedralGoalControl.h"


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

#include "dev/btietz/multiTerrain_OC/OctahedralComplex.h"

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
OctahedralGoalControl::OctahedralGoalControl(JSONGoalControl::Config config,	
                                                std::string args,
                                                std::string resourcePath) :
JSONGoalControl(config, args, resourcePath)
{
    // Path and filename handled by base class
    
}

OctahedralGoalControl::~OctahedralGoalControl()
{
}

void OctahedralGoalControl::onSetup(BaseSpineModelLearning& subject)
{
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
    
    const OctahedralComplex* ocSubject = tgCast::cast<BaseSpineModelLearning, OctahedralComplex>(subject);
    setupSaddleControllers(ocSubject);
    
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

void OctahedralGoalControl::onStep(BaseSpineModelLearning& subject, double dt)
{
    m_updateTime += dt;
    if (m_updateTime >= m_config.controlTime)
    {
#if (1)
    #if (0)
        std::vector<double> desComs = getFeedback(subject);
    #else
        const BaseSpineModelGoal* goalSubject = tgCast::cast<BaseSpineModelLearning,  BaseSpineModelGoal>(subject);
        std::vector<double> desComs = getGoalFeedback(goalSubject);
    #endif // Terrain feedback vs goal feedback
#else 
    #if (1)
        const BaseSpineModelGoal* goalSubject = tgCast::cast<BaseSpineModelLearning,  BaseSpineModelGoal>(subject);
        setGoalTensions(goalSubject);
    #endif
        std::size_t numControllers = subject.getNumberofMuslces() * 3;
        
        double descendingCommand = 0.0;
        std::vector<double> desComs (numControllers, descendingCommand);
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
    }
    
    double currentHeight = subject.getSegmentCOM(m_config.segmentNumber)[1];
    
    /// @todo add to config
    if (currentHeight > 25 || currentHeight < 1.0)
    {
		/// @todo if bogus, stop trial (reset simulation)
		bogus = true;
		throw std::runtime_error("Height out of range");
	}
}

void OctahedralGoalControl::onTeardown(BaseSpineModelLearning& subject)
{
    scores.clear();
    // @todo - check to make sure we ran for the right amount of time
    
    std::vector<double> finalConditions = subject.getSegmentCOM(m_config.segmentNumber);
    
    const double newX = finalConditions[0];
    const double newZ = finalConditions[2];
    const double oldX = initConditions[0];
    const double oldZ = initConditions[2];
    
    const BaseSpineModelGoal* goalSubject = tgCast::cast<BaseSpineModelLearning, BaseSpineModelGoal>(subject);
    
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
    
    for(size_t i = 0; i < m_allControllers.size(); i++)
    {
        delete m_saddleControllers[i];
    }
    m_saddleControllers.clear(); 
}

void OctahedralGoalControl::setupSaddleControllers(const OctahedralComplex* subject)
{
        
    const std::vector <tgSpringCableActuator*> allSaddleMuscles = subject->getSaddleMuscles();
    
    for (std::size_t i = 0; i < allSaddleMuscles.size(); i++)
    {

        tgPIDController::Config config(20000.0, 0.0, 5.0, true); // Non backdrivable
        tgImpedanceController* p_ipc = new tgImpedanceController( m_config.tensFeedback,
                                                                m_config.kPosition,
                                                                0.0);
        
        // Impedance control only, no sine waves
        tgSCASineControl* pStringControl = new tgSCASineControl(m_config.controlTime,
                                                                p_ipc,
                                                                config,
                                                                0.0,
                                                                0.0,
                                                                0.0,
                                                                0.0,
                                                                allSaddleMuscles[i]->getCurrentLength());

        allSaddleMuscles[i]->attach(pStringControl);
        
        m_saddleControllers.push_back(pStringControl);
    }
    
}


void OctahedralGoalControl::setGoalTensions(const BaseSpineModelGoal* subject)
{
    // Get heading and generate feedback vector
    std::vector<double> currentPosition = subject->getSegmentCOM(m_config.segmentNumber);
    
    assert(currentPosition.size() == 3);
    
    btVector3 currentPosVector(currentPosition[0], currentPosition[1], currentPosition[2]);
    
    btVector3 goalPosition = subject->goalBoxPosition();
    
    btVector3 desiredHeading = (goalPosition - currentPosVector).normalize();
    
    std::vector<double> state;
    state.push_back(desiredHeading.getX());
    state.push_back(desiredHeading.getZ());
    
    assert(state[0] >= -1.0 && state[0] <= 1.0);
    assert(state[1] >= -1.0 && state[1] <= 1.0);
    
    double *inputs = new double[m_config.numStates];
    
    // Rescale to 0 to 1 (consider doing this inside getState
    for (std::size_t i = 0; i < state.size(); i++)
    {
        inputs[i]=state[i] / 2.0 + 0.5;
    }
    
    const int nSeg = subject->getSegments() - 1;
    
    double *output = nn->feedForwardPattern(inputs);
    
    vector<double> actions;
    for(int j=0;j<m_config.numActions;j++)
    {
        actions.push_back(output[j]);
    }

    transformFeedbackActions(actions);
    
    assert(m_config.numActions == m_saddleControllers.size() + m_allControllers.size());
    assert(m_saddleControllers.size() == m_allControllers.size());
    
    const OctahedralComplex* octaSubject = tgCast::cast<BaseSpineModelLearning, OctahedralComplex>(subject);
    const std::vector <tgSpringCableActuator*> allSaddleMuscles = octaSubject->getSaddleMuscles();
    const std::vector <tgSpringCableActuator*> allCPGMuscles = octaSubject->getAllMuscles();
    
    
    for (int j = 0; j < m_saddleControllers.size(); j++)
    {
        double startLength = allSaddleMuscles[j]->getStartLength();
        m_saddleControllers[j]->updateControlLength(actions[j] * startLength + startLength);
    }
    
    for (int i = 0; i < m_allControllers.size(); i++)
    {
        double startLength = allCPGMuscles[i]->getStartLength();
        
        tgCPGCableControl* mCPGController = tgCast::cast<tgCPGActuatorControl, tgCPGCableControl>(m_allControllers[i]);
        mCPGController->updateControlLength(actions[i] *startLength + startLength);
    }
}
