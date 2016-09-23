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
 * @file JSONFeedbackControl.cpp
 * @brief A controller for the template class BaseSpineModelLearning
 * @author Brian Mirletz
 * @version 1.1.0
 * $Id$
 */

#include "JSONQuadFeedbackControl.h"


// Should include tgString, but compiler complains since its been
// included from BaseSpineModelLearning. Perhaps we should move things
// to a cpp over there
#include "core/tgSpringCableActuator.h"
#include "core/tgBasicActuator.h"
#include "controllers/tgImpedanceController.h"
#include "examples/learningSpines/tgCPGActuatorControl.h"
#include "dev/CPG_feedback/tgCPGCableControl.h"

#include "examples/learningSpines/BaseSpineModelLearning.h"
#include "helpers/FileHelpers.h"

#include "learning/AnnealEvolution/AnnealEvolution.h"
#include "learning/Configuration/configuration.h"

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

const string strSpineTag = "spine ";
const string strLeftShoulderTag = "left shoulder ";
const string strLeftHipTag = "left hip ";
// Have to use underscores (arbitrary) for some of these because of the way tags work
const string strLeftFrontLegTag = "left_front_leg ";
const string strLeftHindLegTag = "left_hind_leg ";
const string strRightShoulderTag = "right shoulder ";
const string strRightHipTag = "right hip ";
const string strRightFrontLegTag = "right_front_leg ";
const string strRightHindLegTag = "right_hind_leg ";

JSONQuadFeedbackControl::Config::Config(int ss,
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
										double maxH,
										double minH,
										double hffMin,
										double hffMax) :
JSONCPGControl::Config::Config(ss, tm, om, param, segnum, ct, la, ha,
                                    lp, hp, kt, kp, kv, def, cl, lf, hf),
freqFeedbackMin(ffMin),
freqFeedbackMax(ffMax),
highFreqFeedbackMin(hffMin),
highFreqFeedbackMax(hffMax),
ampFeedbackMin(afMin),
ampFeedbackMax(afMax),
phaseFeedbackMin(pfMin),
phaseFeedbackMax(pfMax),
maxHeight(maxH),
minHeight(minH)
{
    
}
/**
 * Defining the adapters here assumes the controller is around and
 * attached for the lifecycle of the learning runs. I.E. that the setup
 * and teardown functions are used for tgModel
 */
JSONQuadFeedbackControl::JSONQuadFeedbackControl(JSONQuadFeedbackControl::Config config,	
                                                std::string args,
                                                std::string resourcePath) :
JSONCPGControl(config, args, resourcePath),
m_config(config)
{
    // Path and filename handled by base class
    
}

JSONQuadFeedbackControl::~JSONQuadFeedbackControl()
{
    delete nn;
}

void JSONQuadFeedbackControl::onSetup(BaseSpineModelLearning& subject)
{
	// spine, 4 legs, 4 hips/shoulers
	n_bodyParts = 9;

	m_pCPGSys = new CPGEquationsFB(100);

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
    
    std::cout << nodeVals << std::endl;
    
    nodeVals = nodeVals.get("params", "UTF-8");
    edgeVals = edgeVals.get("params", "UTF-8");
    
	// parameters for high CPG to high CPG coupling
    Json::Value highEdgeVals = root.get("highVals", "UTF-8");
    highEdgeVals = highEdgeVals.get("params", "UTF-8");

	// parameters for high CPG to low CPG coupling
    Json::Value highLowEdgeVals = root.get("hLowVals", "UTF-8");
	highLowEdgeVals = highLowEdgeVals.get("params", "UTF-8");

	// This is kinda nasty and out of place to put here, but I need to initialize
	// some important variables.
    std::vector <tgSpringCableActuator*> spineMuscles = subject.find<tgSpringCableActuator> (strSpineTag);
	std::vector <tgSpringCableActuator*> leftShoulderMuscles = subject.find<tgSpringCableActuator> (strLeftShoulderTag);
	std::vector <tgSpringCableActuator*> leftFrontLegMuscles = subject.find<tgSpringCableActuator> (strLeftFrontLegTag);
	n_muscSpine = spineMuscles.size();
	n_muscHip = leftShoulderMuscles.size();
	n_muscLeg = leftFrontLegMuscles.size();

    array_4D edgeParams = scaleEdgeActions(edgeVals);
    array_2D nodeParams = scaleNodeActions(nodeVals);
	array_4D highEdgeParams = scaleHighEdgeActions(highEdgeVals);

    setupCPGs(subject, nodeParams, edgeParams);
	setupHighCPGs(nodeParams, highEdgeParams, highLowEdgeVals);
    
    Json::Value feedbackParams = root.get("feedbackVals", "UTF-8");
    feedbackParams = feedbackParams.get("params", "UTF-8");
    
    // Setup neural network
    m_config.numStates = feedbackParams.get("numStates", "UTF-8").asInt();
    m_config.numActions = feedbackParams.get("numActions", "UTF-8").asInt();
    //m_config.numHidden = feedbackParams.get("numHidden", "UTF-8").asInt();
    
    std::string nnFile = controlFilePath + feedbackParams.get("neuralFilename", "UTF-8").asString();
    
    nn = new neuralNetwork(m_config.numStates, m_config.numStates*2, m_config.numActions);
    
    nn->loadWeights(nnFile.c_str());
    
    initConditions = subject.getSegmentCOM(m_config.segmentNumber);
    for (int i = 0; i < initConditions.size(); i++)
    {
        std::cout << initConditions[i] << " ";
    }
    std::cout << std::endl;
#ifdef LOGGING // Conditional compile for data logging    
    m_dataObserver.onSetup(subject);
#endif    
    
#if (0) // Conditional Compile for debug info
    std::cout << *m_pCPGSys << std::endl;
#endif    
    m_updateTime = 0.0;
    bogus = false;
}

void JSONQuadFeedbackControl::onStep(BaseSpineModelLearning& subject, double dt)
{
    m_updateTime += dt;
    if (m_updateTime >= m_config.controlTime)
    {
#if (1)
        std::vector<double> desComs = getFeedback(subject);

#else        
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
    
    /// Max and min heights added to config
    if (currentHeight > m_config.maxHeight || currentHeight < m_config.minHeight)
    {
		/// @todo if bogus, stop trial (reset simulation)
		bogus = true;
		throw std::runtime_error("Height out of range");
	}
}

void JSONQuadFeedbackControl::onTeardown(BaseSpineModelLearning& subject)
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
	//double diff_of_sqrs = (newX-oldX)*(newX-oldX) - (newZ-oldZ)*(newZ-oldZ);
	//const double distanceMoved = diff_of_sqrs >= 0 ? sqrt(diff_of_sqrs) : -sqrt(-diff_of_sqrs);
    
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
    
    std::vector<tgSpringCableActuator* > tmpStrings = subject.find<tgSpringCableActuator> (strSpineTag);
    
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
    
	for (size_t b = 0; b < 9; b++)
	{
		for(size_t i = 0; i < m_controllers[b].size(); i++)
		{
			delete m_controllers[b][i];
		}
    	m_controllers[b].clear();    
	}

	for (size_t i = 0; i < m_highControllers.size(); i++)
	{
		delete m_highControllers[i];
	}
	m_highControllers.clear();
}

void JSONQuadFeedbackControl::setupCPGs(BaseSpineModelLearning& subject, array_2D nodeActions, array_4D edgeActions)
{
    CPGEquationsFB& m_CPGFBSys = *(tgCast::cast<CPGEquations, CPGEquationsFB>(m_pCPGSys));

    std::vector <tgSpringCableActuator*> spineMuscles = subject.find<tgSpringCableActuator> (strSpineTag);
	std::vector <tgSpringCableActuator*> leftShoulderMuscles = subject.find<tgSpringCableActuator> (strLeftShoulderTag);
	std::vector <tgSpringCableActuator*> leftHipMuscles = subject.find<tgSpringCableActuator> (strLeftHipTag);
	std::vector <tgSpringCableActuator*> leftFrontLegMuscles = subject.find<tgSpringCableActuator> (strLeftFrontLegTag);
	std::vector <tgSpringCableActuator*> leftHindLegMuscles = subject.find<tgSpringCableActuator> (strLeftHindLegTag);
	std::vector <tgSpringCableActuator*> rightShoulderMuscles = subject.find<tgSpringCableActuator> (strRightShoulderTag);
	std::vector <tgSpringCableActuator*> rightHipMuscles = subject.find<tgSpringCableActuator> (strRightHipTag);
	std::vector <tgSpringCableActuator*> rightFrontLegMuscles = subject.find<tgSpringCableActuator> (strRightFrontLegTag);
	std::vector <tgSpringCableActuator*> rightHindLegMuscles = subject.find<tgSpringCableActuator> (strRightHindLegTag);
    
	std::vector <tgSpringCableActuator*> allMuscles[9] = { spineMuscles, leftShoulderMuscles, leftFrontLegMuscles, rightShoulderMuscles, rightFrontLegMuscles, leftHipMuscles, leftHindLegMuscles, rightHipMuscles, rightHindLegMuscles };

	// which muscle groups get which node params
	//const size_t node_assignment[n_bodyParts] = { 0, 1, 1, 2, 2, 3, 3, 4, 4 };
	for (std::size_t b = 0; b < n_bodyParts; b++)
	{
		for (std::size_t i = 0; i < allMuscles[b].size(); i++)
		{

			tgPIDController::Config config(20000.0, 0.0, 5.0, true); // Non backdrivable
			tgCPGCableControl* pStringControl = new tgCPGCableControl(config);

			allMuscles[b][i]->attach(pStringControl);
			
			// need to make a 1 row 2D array so we can pass it into assignNodeNumber.
			// 	using nodeActions will not work. (notice subtle name difference) ~B
			array_2D nodeAction(boost::extents[1][nodeActions.shape()[1]]);
			for (std::size_t a = 0; a < nodeActions.shape()[1]; a++)
				nodeAction[0][a] = nodeActions[(b+1)/2 + 5][a];
				//nodeAction[0][a] = nodeActions[node_assignment[b]+5][a];
				// Why b+5? Because node params 0-4 have already been assigned to high level CPGs (above)
			
			// assign node number
			pStringControl->assignNodeNumberFB(m_CPGFBSys, nodeAction);
			
			m_controllers[b].push_back(pStringControl);
		}
	} 

	// The starting index (x and y) for the edgeActions matrix
	// Remember this 4D matrix contains a block matrix
	size_t actionsStart = 0;
    // Then determine connectivity and setup string
	for (size_t b = 0; b < n_bodyParts; b++)
	{
		for (std::size_t i = 0; i < m_controllers[b].size(); i++)
		{
			// Have fun understanding this. It was fun to write. :)
			// Hint: numMuscles is how many muscles exist in body part b
			size_t numMuscles = b == 0 ? n_muscSpine : (b % 2 ? n_muscHip : n_muscLeg);

			array_4D edgeActionsForBodyPart(boost::extents[3][numMuscles][numMuscles][edgeActions.shape()[3]]);
			// I'm doing a nasty copy. boost's documentation for copying a subarray is too confusing.
			for (size_t x = 0; x < 3; x++) for (size_t y = 0; y < numMuscles; y++) for (size_t z = 0; z < numMuscles; z++) for (size_t w = 0; w < edgeActions.shape()[3]; w++)
				edgeActionsForBodyPart[x][y][z][w] = edgeActions[x][y+actionsStart][z+actionsStart][w];

			tgCPGActuatorControl * const pStringInfo = m_controllers[b][i];
			assert(pStringInfo != NULL);
			pStringInfo->setConnectivity(m_controllers[b], edgeActionsForBodyPart);
			
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
		// I love one-liners. ~BG
		actionsStart += b == 0 ? 16 : (b % 2 ? n_muscHip : n_muscLeg);
	}
}

void JSONQuadFeedbackControl::setupHighCPGs(array_2D nodeActions, array_4D highEdgeActions, Json::Value highLowEdgeActions)
{
    CPGEquationsFB& m_CPGFBSys = *(tgCast::cast<CPGEquations, CPGEquationsFB>(m_pCPGSys));

	// Make higher level CPGs, give them node params
	for (std::size_t b = 5; b < 10; b++)
	{
        tgPIDController::Config config(20000.0, 0.0, 5.0, true); // Non backdrivable
        tgCPGCableControl* pStringControl = new tgCPGCableControl(config);

		// need to make a 1 row 2D array so we can pass it into assignNodeNumber.
		// 	using nodeActions will not work. (notice subtle name difference) ~B
		array_2D nodeAction(boost::extents[1][nodeActions.shape()[1]]);
		for (std::size_t a = 0; a < nodeActions.shape()[1]; a++)
			nodeAction[0][a] = nodeActions[b][a];

		// must give node number, to make things easier and stuff
		pStringControl->assignNodeNumberFB(m_CPGFBSys, nodeAction);

		m_highControllers.push_back(pStringControl);
	}
	// So the higher level CPGs are indices 5-9



    // Determine connectivity between higher level CPGs
	for (std::size_t i = 0; i < m_highControllers.size(); i++)
	{
		tgCPGActuatorControl * const pStringInfo = m_highControllers[i];
		assert(pStringInfo != NULL);
		// TODO make sure this works
		pStringInfo->setConnectivity(m_highControllers, highEdgeActions);
	}




	// Now couple high level with low level

    double lowerLimit = m_config.lowPhase;
    double upperLimit = m_config.highPhase;
    double range = upperLimit - lowerLimit;

    Json::Value::iterator edgeIt = highLowEdgeActions.end();

	// Number of node to start at (Note that nodes for low-level CPGs are added first)
	int nodeStart = 0;
	// iterate through high-level CPGs
	for (size_t b = 0; b < 5; b++)
	{
		edgeIt--;

		std::vector<int> connectivityList;
		std::vector<double> weights;
		std::vector<double> phases;
		
		Json::Value param = *edgeIt;
		assert(param.size() == 2);
															
		// n is number of muscles for body part b
		int n = b == 0 ? n_muscSpine : n_muscHip + n_muscLeg;

										// !
		for (size_t i = 0; i < n; i++, nodeStart++)
		{
			connectivityList.push_back(nodeStart);
			weights.push_back(param[0].asDouble());
			phases.push_back(param[1].asDouble() * (range) + lowerLimit);
		}
		
		int highCPGnode = b + n_muscSpine + n_muscHip*4 + n_muscLeg*4;
		m_CPGFBSys.defineConnections(highCPGnode, connectivityList, weights, phases);
	}

	//assert(nodeStart == 16+10*8+5);

	// TODO?
	assert(highLowEdgeActions.begin() == edgeIt);
}

/*
void JSONQuadFeedbackControl::setupHighCouplings(array_4D highEdgeActions)
{
    // Determine connectivity between higher level CPGs
	for (std::size_t i = 0; i < m_highControllers.size(); i++)
	{
		tgCPGActuatorControl * const pStringInfo = m_highControllers[i];
		assert(pStringInfo != NULL);
		pStringInfo->setConnectivity(m_highControllers, highEdgeActions);
		
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

void JSONQuadFeedbackControl::setupHighLowCouplings(Json::Value highLowEdgeActions)
{
	// Now couple high level with low level
	// 											list of other nodenums, eq size list of weights/phases
    //m_pCPGSystem->defineConnections(m_nodeNumber, connectivityList, weights, phases);

    double lowerLimit = m_config.lowPhase;
    double upperLimit = m_config.highPhase;
    double range = upperLimit - lowerLimit;

    CPGEquationsFB& m_CPGFBSys = *(tgCast::cast<CPGEquations, CPGEquationsFB>(m_pCPGSys));

    Json::Value::iterator edgeIt = highLowEdgeActions.end();

	// Number of node to start at (Note that nodes 0-4 are high CPGs)
	int nodeStart = 5;
	for (size_t b = 0; b < 5; b++)
	{
		edgeIt--;

		std::vector<int> connectivityList;
		std::vector<double> weights;
		std::vector<double> phases;
		
		Json::Value param = *edgeIt;
		assert(param.size() == 2);
															
		// Nasty brute forcing. Don't try this at home. Shield your eyes if necessary.
		int n = b == 0 ? 16 : 20;
		// 20 because each hip/shoulder + leg pair has 20 total strings
										// !
		for (size_t i = 0; i < n; i++, nodeStart++)
		{
			connectivityList.push_back(nodeStart);
			weights.push_back(param[0].asDouble());
			phases.push_back(param[1].asDouble());
		}
		
		m_CPGFBSys.defineConnections(b, connectivityList, weights, phases);
	}

	assert(nodeStart == 16+10*8+5);

	// TODO?
	assert(highLowEdgeActions.begin() == edgeIt);
}
*/

array_4D JSONQuadFeedbackControl::scaleEdgeActions  
                            (Json::Value edgeParam)
{
	// This creates a gigantic 4D array where the 2nd and 3rd dimensions
	// form a block matrix (wikipedia it) with blocks along the diagonal. 
	// Each block corresponds to the couplings among muscles in the same body part.
	// So, the first block is couplings between spine muscles only, the next block
	// is couplings between left front shoulder muscles only, etc.

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
 
	// 16 is the number of muscles per segment in the spine
	// I don't know how to find this dynamically if this number changes
	// This might be findable using tags, but we'll leave this as an exercise for the reader
	int musc = 16;
	for (size_t b = 0; b < n_bodyParts; b++)
	{
		while (i < m_config.segmentSpan)
		{
			// Only the spine will have couplings with muscles on other rigid bodies
			// For legs and hips and stuff, only the muscles on the same rigid body
			// can couple
			if (b != 0 && i != 1)
			{
				i++;
				continue;
			}

			while(j < musc)
			{
				while(k < musc)
				{
					//cout << b << " " << i << " " << j << " " << k << " " << musc << endl;
					if (edgeIt == edgeParam.begin())
					{
						std::cout << "ran out before table populated!"
						<< std::endl;
						cout << count << endl;
						cout << b << " " << i << " " << j << " " << k << " " << musc << endl;
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
							cout << b << " " << i << " " << j << " " << k << " " << musc << endl;
							edgeIt--;
							Json::Value edgeParam1 = *edgeIt;
							assert(edgeParam1.size() == 2);
							// Weight from 0 to 1
							actionList[i][j][k][0] = edgeParam1[0].asDouble();
							//std::cout << actionList[i][j][k][0] << " ";
							// Phase offset from -pi to pi
							actionList[i][j][k][1] = edgeParam1[1].asDouble() * 
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
			j = musc - (b > 0 ? (b % 2 ? n_muscHip : n_muscLeg) : 16);
			k = j;
			i++;
		}
		j = musc;
		k = musc;
		i = 0;
		musc += b % 2 ? n_muscHip : n_muscLeg;
	}
    
    std::cout<< "Params used: " << count << std::endl;
    
    assert(edgeParam.begin() == edgeIt);
    
    return actionList;
}

array_4D JSONQuadFeedbackControl::scaleHighEdgeActions (Json::Value highEdgeParam)
{
    assert(highEdgeParam[0].size() == 2);
    
    double lowerLimit = m_config.lowPhase;
    double upperLimit = m_config.highPhase;
    double range = upperLimit - lowerLimit;
    
	// This is a 4d array, but we don't really worry about 1st dimension
	// since these higher CPGs aren't attached to any rigid bodies
	// Using 5 by 5 because there are 5 higher level CPGs
	// Maybe someday someone will program a more elegant solution... Nah, that won't happen.
    array_4D actionList(boost::extents[m_config.segmentSpan]
										[5]
										[5]
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
    Json::Value::iterator highEdgeIt = highEdgeParam.end();
    
    int count = 0;
    
	while(j < 5)
	{
		while(k < 5)
		{
			if (j == k)
			{
				// std::cout << "Skipped identical muscle" << std::endl;
				//Skip since its the same muscle
			}
			else
			{
				if (highEdgeIt == highEdgeParam.begin())
				{
					std::cout << "ran out before table populated!"
					<< std::endl;
					/// @todo consider adding exception here
					break;
				}
				else
				{
					//cout << j << " " << k << endl;
					highEdgeIt--;
					Json::Value edgeParam1 = *highEdgeIt;
					assert(edgeParam1.size() == 2);
					// Weight from 0 to 1
					while (i < m_config.segmentSpan)
					{
						actionList[i][j][k][0] = edgeParam1[0].asDouble();
						//std::cout << actionList[i][j][k][0] << " ";
						// Phase offset from -pi to pi
						actionList[i][j][k][1] = edgeParam1[1].asDouble() * 
												(range) + lowerLimit;
						//std::cout <<  actionList[i][j][k][1] << std::endl;
						i++;
					}
					i = 0;
					count++;
				}
			}
			k++;
		}
		j++;
		k = j;
	}
    
    std::cout<< "Params used: " << count << std::endl;
    
    assert(highEdgeParam.begin() == highEdgeIt);
    
    return actionList;
}

array_2D JSONQuadFeedbackControl::scaleNodeActions (Json::Value actions)
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
		// Recall that high CPG nodes are added last
		if (i > n_muscSpine + n_muscHip*4 + n_muscLeg*4)
		{
			// We use separate parameters for the frequency of the high
			// CPGs since they control entire body parts, y'know?
			limits[0][2] = m_config.highFreqFeedbackMin;
			limits[1][2] = m_config.highFreqFeedbackMax;
		}
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

std::vector<double> JSONQuadFeedbackControl::getFeedback(BaseSpineModelLearning& subject)
{
    // Placeholder
    std::vector<double> feedback;
    
    const std::vector<tgSpringCableActuator*>& spineCables = subject.find<tgSpringCableActuator> (strSpineTag);
	const std::vector<tgSpringCableActuator*>& leftShoulderCables = subject.find<tgSpringCableActuator> (strLeftShoulderTag);
	const std::vector<tgSpringCableActuator*>& leftHipCables = subject.find<tgSpringCableActuator> (strLeftHipTag);
	const std::vector<tgSpringCableActuator*>& leftFrontLegCables = subject.find<tgSpringCableActuator> (strLeftFrontLegTag);
	const std::vector<tgSpringCableActuator*>& leftHindLegCables = subject.find<tgSpringCableActuator> (strLeftHindLegTag);
	const std::vector<tgSpringCableActuator*>& rightShoulderCables = subject.find<tgSpringCableActuator> (strRightShoulderTag);
	const std::vector<tgSpringCableActuator*>& rightHipCables = subject.find<tgSpringCableActuator> (strRightHipTag);
	const std::vector<tgSpringCableActuator*>& rightFrontLegCables = subject.find<tgSpringCableActuator> (strRightFrontLegTag);
	const std::vector<tgSpringCableActuator*>& rightHindLegCables = subject.find<tgSpringCableActuator> (strRightHindLegTag);
    
	const std::vector <tgSpringCableActuator*> allCables[9] = { spineCables, leftShoulderCables, leftFrontLegCables, rightShoulderCables, rightFrontLegCables, leftHipCables, leftHindLegCables, rightHipCables, rightHindLegCables };

    double *inputs = new double[m_config.numStates];
    
	for (size_t b = 0; b < 9; b++)
	{
    	std::size_t n = allCables[b].size();
		for(std::size_t i = 0; i != n; i++)
		{
			std::vector< std::vector<double> > actions;
			
			const tgSpringCableActuator& cable = *(allCables[b][i]);
			std::vector<double > state = getCableState(cable);
			
			// Rescale to 0 to 1 (consider doing this inside getState
			for (std::size_t i = 0; i < state.size(); i++)
			{
				inputs[i]=state[i] / 2.0 + 0.5;
			}
			
			double *output = nn->feedForwardPattern(inputs);
			vector<double> tmpAct;
			for(int j=0;j<m_config.numActions;j++)
			{
				tmpAct.push_back(output[j]);
			}
			actions.push_back(tmpAct);

			std::vector<double> cableFeedback = transformFeedbackActions(actions);
			
			feedback.insert(feedback.end(), cableFeedback.begin(), cableFeedback.end());
		}
	}
    
	// inputting 0 for now
    std::size_t n = m_highControllers.size();
	for (size_t i = 0; i != n; i++)
	{
		std::vector< std::vector<double> > actions;
		
		// Rescale to 0 to 1 (consider doing this inside getState
		for (std::size_t i = 0; i < m_config.numStates; i++)
		{
			inputs[i] = 0;
		}
		
		double *output = nn->feedForwardPattern(inputs);
		vector<double> tmpAct;
		for(int j=0;j<m_config.numActions;j++)
		{
			tmpAct.push_back(output[j]);
		}
		actions.push_back(tmpAct);

		std::vector<double> cableFeedback = transformFeedbackActions(actions);
		
		feedback.insert(feedback.end(), cableFeedback.begin(), cableFeedback.end());
	}

    return feedback;
}

std::vector<double> JSONQuadFeedbackControl::getCableState(const tgSpringCableActuator& cable)
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

std::vector<double> JSONQuadFeedbackControl::transformFeedbackActions(std::vector< std::vector<double> >& actions)
{
	// Placeholder
	std::vector<double> feedback;
    
    // Leave in place for generalization later
    const std::size_t numControllers = 1;
    const std::size_t numActions = m_config.numActions;
    
    assert( actions.size() == numControllers);
    assert( actions[0].size() == numActions);
    
    // Scale values back to -1 to +1
    for( std::size_t i = 0; i < numControllers; i++)
    {
        for( std::size_t j = 0; j < numActions; j++)
        {
            feedback.push_back(actions[i][j] * 2.0 - 1.0);
        }
    }
    
	return feedback;
}

