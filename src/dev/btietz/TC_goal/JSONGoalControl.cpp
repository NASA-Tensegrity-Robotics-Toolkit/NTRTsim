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

#include "JSONGoalControl.h"


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

JSONGoalControl::Config::Config(int ss,
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
                                        double hf,
                                        double ffMin,
                                        double ffMax,
                                        double afMin,
                                        double afMax,
                                        double pfMin,
                                        double pfMax,
                                        double tf) :
JSONCPGControl::Config::Config(ss, tm, om, param, segnum, ct, la, ha,
                                    lp, hp, kt, kp, kv, def, cl, lf, hf),
freqFeedbackMin(ffMin),
freqFeedbackMax(ffMax),
ampFeedbackMin(afMin),
ampFeedbackMax(afMax),
phaseFeedbackMin(pfMin),
phaseFeedbackMax(pfMax),
tensFeedback(tf)
{
    
}
/**
 * Defining the adapters here assumes the controller is around and
 * attached for the lifecycle of the learning runs. I.E. that the setup
 * and teardown functions are used for tgModel
 */
JSONGoalControl::JSONGoalControl(JSONGoalControl::Config config,	
                                                std::string args,
                                                std::string resourcePath) :
JSONCPGControl(config, args, resourcePath),
m_config(config),
nn(NULL),
nn_goal(NULL)
{
    // Path and filename handled by base class
    
}

JSONGoalControl::~JSONGoalControl()
{
}

void JSONGoalControl::onSetup(BaseSpineModelLearning& subject)
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
    
    // TODO make a function that does this based on a NN pointer and a string
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

void JSONGoalControl::onStep(BaseSpineModelLearning& subject, double dt)
{
    m_updateTime += dt;
    if (m_updateTime >= m_config.controlTime)
    {
#if (1) // Goal and cable

    std::vector<double> desComs = getFeedback(subject);

    const BaseSpineModelGoal* goalSubject = tgCast::cast<BaseSpineModelLearning,  BaseSpineModelGoal>(subject);
    std::vector<double> desComsSet2 = getGoalFeedback(goalSubject);
    
    std::size_t n = desComs.size();
    assert(n == desComsSet2.size());
    
    for (std::size_t i = 0; i < n; i++)
    {
        desComs[i] += desComsSet2[i];
    }
    
#elif (1) // Just goal
    const BaseSpineModelGoal* goalSubject = tgCast::cast<BaseSpineModelLearning,  BaseSpineModelGoal>(subject);
    std::vector<double> desComs = getGoalFeedback(goalSubject);
#else // Nothing
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

void JSONGoalControl::onTeardown(BaseSpineModelLearning& subject)
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
    
    delete nn;
    delete nn_goal;
}

void JSONGoalControl::setupCPGs(BaseSpineModelLearning& subject, array_2D nodeActions, array_4D edgeActions)
{
	    
    std::vector <tgSpringCableActuator*> allMuscles = subject.getAllMuscles();
    
    CPGEquationsFB& m_CPGFBSys = *(tgCast::cast<CPGEquations, CPGEquationsFB>(m_pCPGSys));
    
    for (std::size_t i = 0; i < allMuscles.size(); i++)
    {

        tgPIDController::Config config(20000.0, 0.0, 5.0, true); // Non backdrivable
        tgCPGCableControl* pStringControl = new tgCPGCableControl(config);

        allMuscles[i]->attach(pStringControl);
        
        // First assign node numbers
        pStringControl->assignNodeNumberFB(m_CPGFBSys, nodeActions);
        
        m_allControllers.push_back(pStringControl);
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

array_2D JSONGoalControl::scaleNodeActions (Json::Value actions)
{
    std::size_t numControllers = actions.size();
    std::size_t numActions = actions[0].size();
    
    array_2D nodeActions(boost::extents[numControllers][numActions]);
    
    array_2D limits(boost::extents[2][numActions]);
    
    // Check if we need to update limits
    assert(numActions == 5);
    
	limits[0][0] = m_config.lowFreq;
	limits[1][0] = m_config.highFreq;
	limits[0][1] = m_config.lowAmp;
	limits[1][1] = m_config.highAmp;
    limits[0][2] = m_config.freqFeedbackMin;
    limits[1][2] = m_config.freqFeedbackMax;
    limits[0][3] = m_config.ampFeedbackMin;
    limits[1][3] = m_config.ampFeedbackMax;
    limits[0][4] = m_config.phaseFeedbackMin;
    limits[1][4] = m_config.phaseFeedbackMax;
    
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

std::vector<double> JSONGoalControl::getGoalFeedback(const BaseSpineModelGoal* subject)
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
    
    // Duplicate the actions across segments
    for (int i = 0; i != nSeg; i++)
    {
        for(int j=0;j<m_config.goalActions;j++)
        {
            actions.push_back(output[j]);
        }
    }


    transformFeedbackActions(actions);
    
    return actions;
}

std::vector<double> JSONGoalControl::getFeedback(BaseSpineModelLearning& subject)
{
    // Placeholder
    std::vector<double> feedback;
    
    const std::vector<tgSpringCableActuator*>& allCables = subject.getAllMuscles();
    
    double *inputs = new double[m_config.numStates];
    
    std::size_t n = allCables.size();
    for(std::size_t i = 0; i != n; i++)
    {        
        const tgSpringCableActuator& cable = *(allCables[i]);
        std::vector<double > state = getCableState(cable);
        
        // Rescale to 0 to 1 (consider doing this inside getState
        for (std::size_t i = 0; i < state.size(); i++)
        {
            inputs[i]=state[i];
        }
        
        double *output = nn->feedForwardPattern(inputs);
        vector<double> actions;
        for(int j=0;j<m_config.numActions;j++)
        {
            actions.push_back(output[j]);
        }

        transformFeedbackActions(actions);
        
        feedback.insert(feedback.end(), actions.begin(), actions.end());
    }
    
    
    return feedback;
}

std::vector<double> JSONGoalControl::getCableState(const tgSpringCableActuator& cable)
{
	// For each string, scale value from -1 to 1 based on initial length or max tension of motor
    
    std::vector<double> state;
    
    // Scale length by starting length
    const double startLength = cable.getStartLength();
    state.push_back((cable.getCurrentLength() - startLength) / startLength);
    
    const double maxTension = cable.getConfig().maxTens;
    state.push_back((cable.getTension() - maxTension / 2.0) / maxTension);
    
	return state;
}

void JSONGoalControl::transformFeedbackActions(std::vector<double> & actions)
{
#if (0) // Only true if actions are applied to all segments
    assert( actions.size() == numActions);
#endif
    // Scale values back to -1 to +1
    for( std::size_t i = 0; i < actions.size(); i++)
    {
        actions[i] = actions[i] * 2.0 - 1.0;
    }
}

double JSONGoalControl::calculateDistanceMoved(const BaseSpineModelGoal* subject) const
{
    std::vector<double> finalConditions = subject->getSegmentCOM(m_config.segmentNumber);
  
    const btVector3 goalPos = subject->goalBoxPosition();
    
    std::cout << goalPos << std::endl;
    
    double x= finalConditions[0] - goalPos.getX();
    double z= finalConditions[2] - goalPos.getZ();
    double distanceNew=sqrt(x*x + z*z);
    double xx=initConditions[0]-goalPos.getX();
    double zz=initConditions[2]-goalPos.getZ();
    double distanceOld=sqrt(xx*xx + zz*zz);
    double distanceMoved=distanceOld-distanceNew;

    //If you want to calculate only the distance moved independent of the target:
//  distanceMoved=sqrt((x-xx)*(x-xx)+(z-zz)*(z-zz));

    return distanceMoved;
}
