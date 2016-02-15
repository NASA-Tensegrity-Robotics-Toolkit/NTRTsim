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
 * @file JSONStatsFeedbackControl.cpp
 * @brief A controller for the template class BaseSpineModelLearning. 
 * @Includes more metrics, such as center of mass of entire structure.
 * @author Brian Mirletz, Dawn Hustig-Schultz
 * @version 1.1.0
 * $Id$
 */

#include "JSONMetricsFeedbackControl.h"


// Should include tgString, but compiler complains since its been
// included from BaseSpineModelLearning. Perhaps we should move things
// to a cpp over there
#include "core/tgSpringCableActuator.h"
#include "core/tgBasicActuator.h"
#include "controllers/tgImpedanceController.h"
#include "examples/learningSpines/tgCPGActuatorControl.h"
#include "dev/CPG_feedback/tgCPGCableControl.h"

#include "dev/dhustigschultz/BigPuppy_SpineOnly_Stats/BaseQuadModelLearning.h"
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

JSONMetricsFeedbackControl::Config::Config(int ss,
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
					double minH) :
JSONQuadCPGControl::Config::Config(ss, tm, om, param, segnum, ct, la, ha,
                                    lp, hp, kt, kp, kv, def, cl, lf, hf),
freqFeedbackMin(ffMin),
freqFeedbackMax(ffMax),
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
JSONMetricsFeedbackControl::JSONMetricsFeedbackControl(JSONMetricsFeedbackControl::Config config,	
                                                std::string args,
                                                std::string resourcePath) :
JSONQuadCPGControl(config, args, resourcePath),
m_config(config)
{
    // Path and filename handled by base class
    
}

JSONMetricsFeedbackControl::~JSONMetricsFeedbackControl()
{
    delete nn;
}

void JSONMetricsFeedbackControl::onSetup(BaseQuadModelLearning& subject)
{
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
    
    array_4D edgeParams = scaleEdgeActions(edgeVals);
    array_2D nodeParams = scaleNodeActions(nodeVals);

    setupCPGs(subject, nodeParams, edgeParams);
    
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
    m_totalTime = 0.0; //For metrics. 
    bogus = false;

    metrics.clear();

    //Getting the center of mass of the entire structure. 
    std::vector<double> structureCOM = subject.getCOM(m_config.segmentNumber);

    for(std::size_t i=0; i<3; i++)
    {
	metrics.push_back(structureCOM[i]);
    }
    
    //"metrics" is a new section of the controller's JSON file that is 
    //added in the getNewFile function in evolution_job_master.py 
    Json::Value prevMetrics = root.get("metrics", Json::nullValue);

    Json::Value subMetrics;
    subMetrics["initial COM x"] = metrics[0];
    subMetrics["initial COM y"] = metrics[1];
    subMetrics["initial COM z"] = metrics[2];
    
    prevMetrics.append(subMetrics);
    root["metrics"] = prevMetrics;
    
    ofstream payloadLog;
    payloadLog.open(controlFilename.c_str(),ofstream::out);
    
    payloadLog << root << std::endl;
}

void JSONMetricsFeedbackControl::onStep(BaseQuadModelLearning& subject, double dt)
{
    m_updateTime += dt;
    m_totalTime += dt;
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
    //every 100 steps, get the COM and tensions of active muscles and store them in the JSON file.
 if(1){
	    static int count = 0;
	    if(count > 100) {
		m_timeStep.push_back(m_totalTime);

		//Getting the center of mass of the entire structure. 
		std::vector<double> structureCOM = subject.getCOM(m_config.segmentNumber);
		m_quadCOM.push_back(structureCOM);

		std::vector<tgSpringCableActuator* > tmpStrings = subject.find<tgSpringCableActuator> ("spine ");

		m_tensions.clear();
		for(std::size_t i=0; i<tmpStrings.size(); i++)
		{
		    double tension = tmpStrings[i]->getTension();
		    m_tensions.push_back(tension);
		
		}

		//Yes, I know, there are much better ways to do this. See note at line 665. I'll make this better after IROS.
		m_muscleTensionZero.push_back(m_tensions[0]);
    		m_muscleTensionOne.push_back(m_tensions[1]);
    		m_muscleTensionTwo.push_back(m_tensions[2]);
    		m_muscleTensionThree.push_back(m_tensions[3]);
    		m_muscleTensionFour.push_back(m_tensions[4]);
    		m_muscleTensionFive.push_back(m_tensions[5]);
    		m_muscleTensionSix.push_back(m_tensions[6]);
    		m_muscleTensionSeven.push_back(m_tensions[7]);
    		m_muscleTensionEight.push_back(m_tensions[8]);
    		m_muscleTensionNine.push_back(m_tensions[9]);
    		m_muscleTensionTen.push_back(m_tensions[10]);
    		m_muscleTensionEleven.push_back(m_tensions[11]);
    		m_muscleTensionTwelve.push_back(m_tensions[12]);
    		m_muscleTensionThirteen.push_back(m_tensions[13]);
    		m_muscleTensionFourteen.push_back(m_tensions[14]);
    		m_muscleTensionFifteen.push_back(m_tensions[15]);
    		m_muscleTensionSixteen.push_back(m_tensions[16]);
    		m_muscleTensionSeventeen.push_back(m_tensions[17]);
    		m_muscleTensionEighteen.push_back(m_tensions[18]);
    		m_muscleTensionNineteen.push_back(m_tensions[19]);
    		m_muscleTensionTwenty.push_back(m_tensions[20]);
    		m_muscleTensionTwentyOne.push_back(m_tensions[21]);
    		m_muscleTensionTwentyTwo.push_back(m_tensions[22]);
    		m_muscleTensionTwentyThree.push_back(m_tensions[23]);
    		m_muscleTensionTwentyFour.push_back(m_tensions[24]);
    		m_muscleTensionTwentyFive.push_back(m_tensions[25]);
    		m_muscleTensionTwentySix.push_back(m_tensions[26]);
    		m_muscleTensionTwentySeven.push_back(m_tensions[27]);
    		m_muscleTensionTwentyEight.push_back(m_tensions[28]);
    		m_muscleTensionTwentyNine.push_back(m_tensions[29]);
    		m_muscleTensionThirty.push_back(m_tensions[30]);
    		m_muscleTensionThirtyOne.push_back(m_tensions[31]);
    		m_muscleTensionThirtyTwo.push_back(m_tensions[32]);
    		m_muscleTensionThirtyThree.push_back(m_tensions[33]);
    		m_muscleTensionThirtyFour.push_back(m_tensions[34]);
    		m_muscleTensionThirtyFive.push_back(m_tensions[35]);
    		m_muscleTensionThirtySix.push_back(m_tensions[36]);
    		m_muscleTensionThirtySeven.push_back(m_tensions[37]);
    		m_muscleTensionThirtyEight.push_back(m_tensions[38]);
    		m_muscleTensionThirtyNine.push_back(m_tensions[39]);
    		m_muscleTensionForty.push_back(m_tensions[40]);
    		m_muscleTensionFortyOne.push_back(m_tensions[41]);
    		m_muscleTensionFortyTwo.push_back(m_tensions[42]);
    		m_muscleTensionFortyThree.push_back(m_tensions[43]);
    		m_muscleTensionFortyFour.push_back(m_tensions[44]);
    		m_muscleTensionFortyFive.push_back(m_tensions[45]);
    		m_muscleTensionFortySix.push_back(m_tensions[46]);
    		m_muscleTensionFortySeven.push_back(m_tensions[47]);
    		m_muscleTensionFortyEight.push_back(m_tensions[48]);
    		m_muscleTensionFortyNine.push_back(m_tensions[49]);
    		m_muscleTensionFifty.push_back(m_tensions[50]);
                m_muscleTensionFiftyOne.push_back(m_tensions[51]);
		
		for(std::size_t i=0; i<tmpStrings.size(); i++)
		{
		    double length = tmpStrings[i]->getCurrentLength();
		    m_lengths.push_back(length);
		}

		m_muscleLengthZero.push_back(m_lengths[0]);
    		m_muscleLengthOne.push_back(m_lengths[1]);
    		m_muscleLengthTwo.push_back(m_lengths[2]);
    		m_muscleLengthThree.push_back(m_lengths[3]);
    		m_muscleLengthFour.push_back(m_lengths[4]);
    		m_muscleLengthFive.push_back(m_lengths[5]);
    		m_muscleLengthSix.push_back(m_lengths[6]);
    		m_muscleLengthSeven.push_back(m_lengths[7]);
    		m_muscleLengthEight.push_back(m_lengths[8]);
    		m_muscleLengthNine.push_back(m_lengths[9]);
    		m_muscleLengthTen.push_back(m_lengths[10]);
    		m_muscleLengthEleven.push_back(m_lengths[11]);
    		m_muscleLengthTwelve.push_back(m_lengths[12]);
    		m_muscleLengthThirteen.push_back(m_lengths[13]);
    		m_muscleLengthFourteen.push_back(m_lengths[14]);
    		m_muscleLengthFifteen.push_back(m_lengths[15]);
    		m_muscleLengthSixteen.push_back(m_lengths[16]);
    		m_muscleLengthSeventeen.push_back(m_lengths[17]);
    		m_muscleLengthEighteen.push_back(m_lengths[18]);
    		m_muscleLengthNineteen.push_back(m_lengths[19]);
    		m_muscleLengthTwenty.push_back(m_lengths[20]);
    		m_muscleLengthTwentyOne.push_back(m_lengths[21]);
    		m_muscleLengthTwentyTwo.push_back(m_lengths[22]);
    		m_muscleLengthTwentyThree.push_back(m_lengths[23]);
    		m_muscleLengthTwentyFour.push_back(m_lengths[24]);
    		m_muscleLengthTwentyFive.push_back(m_lengths[25]);
    		m_muscleLengthTwentySix.push_back(m_lengths[26]);
    		m_muscleLengthTwentySeven.push_back(m_lengths[27]);
    		m_muscleLengthTwentyEight.push_back(m_lengths[28]);
    		m_muscleLengthTwentyNine.push_back(m_lengths[29]);
    		m_muscleLengthThirty.push_back(m_lengths[30]);
    		m_muscleLengthThirtyOne.push_back(m_lengths[31]);
    		m_muscleLengthThirtyTwo.push_back(m_lengths[32]);
    		m_muscleLengthThirtyThree.push_back(m_lengths[33]);
    		m_muscleLengthThirtyFour.push_back(m_lengths[34]);
    		m_muscleLengthThirtyFive.push_back(m_lengths[35]);
    		m_muscleLengthThirtySix.push_back(m_lengths[36]);
   		m_muscleLengthThirtySeven.push_back(m_lengths[37]);
    		m_muscleLengthThirtyEight.push_back(m_lengths[38]);
    		m_muscleLengthThirtyNine.push_back(m_lengths[39]);
    		m_muscleLengthForty.push_back(m_lengths[40]);
    		m_muscleLengthFortyOne.push_back(m_lengths[41]);
    		m_muscleLengthFortyTwo.push_back(m_lengths[42]);
    		m_muscleLengthFortyThree.push_back(m_lengths[43]);
    		m_muscleLengthFortyFour.push_back(m_lengths[44]);
    		m_muscleLengthFortyFive.push_back(m_lengths[45]);
    		m_muscleLengthFortySix.push_back(m_lengths[46]);
    		m_muscleLengthFortySeven.push_back(m_lengths[47]);
    		m_muscleLengthFortyEight.push_back(m_lengths[48]);
    		m_muscleLengthFortyNine.push_back(m_lengths[49]);
    		m_muscleLengthFifty.push_back(m_lengths[50]);
    		m_muscleLengthFiftyOne.push_back(m_lengths[51]);

		count = 0;
	    }
	    else {
		count++;
	    }
    }
}

void JSONMetricsFeedbackControl::onTeardown(BaseQuadModelLearning& subject)
{
    scores.clear();
    metrics.clear();
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
    
    std::vector<tgSpringCableActuator* > tmpStrings = subject.find<tgSpringCableActuator> ("spine ");
    
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

    //Getting the center of mass of the entire structure. 
    std::vector<double> structureCOM = subject.getCOM(m_config.segmentNumber);

    for(std::size_t i=0; i<3; i++)
    {
	metrics.push_back(structureCOM[i]);
    }
    
    std::cout << "Dist travelled " << scores[0] << std::endl;
    
    Json::Value root; // will contain the root value after parsing.
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
    Json::Value prevMetrics = root.get("metrics", Json::nullValue);
    
    Json::Value subScores;
    subScores["distance"] = scores[0];
    subScores["energy"] = scores[1];

    Json::Value subMetrics;
    subMetrics["final COM x"] = metrics[0];
    subMetrics["final COM y"] = metrics[1];
    subMetrics["final COM z"] = metrics[2];
    
    prevScores.append(subScores);
    prevMetrics.append(subMetrics);

    root["scores"] = prevScores;
    root["metrics"] = prevMetrics;
    
    ofstream payloadLog;
    payloadLog.open(controlFilename.c_str(),ofstream::out);
    
    payloadLog << root << std::endl;

    printMetrics();
    
    delete m_pCPGSys;
    m_pCPGSys = NULL;
    
    for(size_t i = 0; i < m_spineControllers.size(); i++)
    {
        delete m_spineControllers[i];
    }
    m_spineControllers.clear();    
}

void JSONMetricsFeedbackControl::setupCPGs(BaseQuadModelLearning& subject, array_2D nodeActions, array_4D edgeActions)
{
	    
    std::vector <tgSpringCableActuator*> spineMuscles = subject.find<tgSpringCableActuator> ("spine ");
    
    CPGEquationsFB& m_CPGFBSys = *(tgCast::cast<CPGEquations, CPGEquationsFB>(m_pCPGSys));
    
    for (std::size_t i = 0; i < spineMuscles.size(); i++)
    {

        tgPIDController::Config config(20000.0, 0.0, 5.0, true); // Non backdrivable
        tgCPGCableControl* pStringControl = new tgCPGCableControl(config);

        spineMuscles[i]->attach(pStringControl);
        
        // First assign node numbers
        pStringControl->assignNodeNumberFB(m_CPGFBSys, nodeActions);
        
        m_spineControllers.push_back(pStringControl);
    }
    
    // Then determine connectivity and setup string
    for (std::size_t i = 0; i < m_spineControllers.size(); i++)
    {
        tgCPGActuatorControl * const pStringInfo = m_spineControllers[i];
        assert(pStringInfo != NULL);
        pStringInfo->setConnectivity(m_spineControllers, edgeActions);
        
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

array_2D JSONMetricsFeedbackControl::scaleNodeActions (Json::Value actions)
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

std::vector<double> JSONMetricsFeedbackControl::getFeedback(BaseQuadModelLearning& subject)
{
    // Placeholder
    std::vector<double> feedback;
    
    const std::vector<tgSpringCableActuator*>& spineCables = subject.find<tgSpringCableActuator> ("spine ");
    
    double *inputs = new double[m_config.numStates];
    
    std::size_t n = spineCables.size();
    for(std::size_t i = 0; i != n; i++)
    {
        std::vector< std::vector<double> > actions;
        
        const tgSpringCableActuator& cable = *(spineCables[i]);
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
    
    
    return feedback;
}

std::vector<double> JSONMetricsFeedbackControl::getCableState(const tgSpringCableActuator& cable)
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

std::vector<double> JSONMetricsFeedbackControl::transformFeedbackActions(std::vector< std::vector<double> >& actions)
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


//Yes, I know. This is an abomination. It was not well thought out, and was as painful to write as it is to read. 
//It was hastily done so that I wouldn't have to make a .csv file by hand for some plots. 
//I want to use two dimensional vectors in the future for all the muscle lengths and tensions, and nested loops for printing the contents. 
//I'll make this change after IROS. 
void JSONMetricsFeedbackControl::printMetrics(){

    //Printing the timesteps for which metrics were gathered: 
    for (int i = 0; i < m_timeStep.size(); ++i) {
        cout << m_timeStep[i] << ',';
    }
    cout << endl;

    //Printing Tensions, to redirect to a .csv (for plotting):
    for (int i = 0; i < m_muscleTensionZero.size(); ++i) {
        cout << m_muscleTensionZero[i];
	if (i < m_muscleTensionZero.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionOne.size(); ++i) {
        cout << m_muscleTensionOne[i];
	if (i < m_muscleTensionOne.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionTwo.size(); ++i) {
        cout << m_muscleTensionTwo[i];
	if (i < m_muscleTensionTwo.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;	

    for (int i = 0; i < m_muscleTensionThree.size(); ++i) {
        cout << m_muscleTensionThree[i];
	if (i < m_muscleTensionThree.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionFour.size(); ++i) {
        cout << m_muscleTensionFour[i];
	if (i < m_muscleTensionFour.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionFive.size(); ++i) {
        cout << m_muscleTensionFive[i];
	if (i < m_muscleTensionFive.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionSix.size(); ++i) {
        cout << m_muscleTensionSix[i];
	if (i < m_muscleTensionSix.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionSeven.size(); ++i) {
        cout << m_muscleTensionSeven[i];
	if (i < m_muscleTensionSeven.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionEight.size(); ++i) {
        cout << m_muscleTensionEight[i];
	if (i < m_muscleTensionEight.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionNine.size(); ++i) {
        cout << m_muscleTensionNine[i];
	if (i < m_muscleTensionNine.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionTen.size(); ++i) {
        cout << m_muscleTensionTen[i];
	if (i < m_muscleTensionTen.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionEleven.size(); ++i) {
        cout << m_muscleTensionEleven[i];
	if (i < m_muscleTensionEleven.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionTwelve.size(); ++i) {
        cout << m_muscleTensionTwelve[i];
	if (i < m_muscleTensionFourteen.size() - 1){
 	    cout<< ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionThirteen.size(); ++i) {
        cout << m_muscleTensionThirteen[i];
	if (i < m_muscleTensionThirteen.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionFourteen.size(); ++i) {
        cout << m_muscleTensionFourteen[i];
	if (i < m_muscleTensionFourteen.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionFifteen.size(); ++i) {
        cout << m_muscleTensionFifteen[i];
	if (i < m_muscleTensionFifteen.size() - 1){
 	    cout<< ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionSixteen.size(); ++i) {
        cout << m_muscleTensionSixteen[i];
	if (i < m_muscleTensionSixteen.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionSeventeen.size(); ++i) {
        cout << m_muscleTensionSeventeen[i];
	if (i < m_muscleTensionSeventeen.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionEighteen.size(); ++i) {
        cout << m_muscleTensionEighteen[i];
	if (i < m_muscleTensionEighteen.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionNineteen.size(); ++i) {
        cout << m_muscleTensionNineteen[i];
	if (i < m_muscleTensionNineteen.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionTwenty.size(); ++i) {
        cout << m_muscleTensionTwenty[i];
	if (i < m_muscleTensionTwenty.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionTwentyOne.size(); ++i) {
        cout << m_muscleTensionTwentyOne[i];
	if (i < m_muscleTensionTwentyOne.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionTwentyTwo.size(); ++i) {
        cout << m_muscleTensionTwentyTwo[i];
	if (i < m_muscleTensionTwentyTwo.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionTwentyThree.size(); ++i) {
        cout << m_muscleTensionTwentyThree[i];
	if (i < m_muscleTensionTwentyThree.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionTwentyFour.size(); ++i) {
        cout << m_muscleTensionTwentyFour[i];
	if (i < m_muscleTensionTwentyFour.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionTwentyFive.size(); ++i) {
        cout << m_muscleTensionTwentyFive[i];
	if (i < m_muscleTensionTwentyFive.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionTwentySix.size(); ++i) {
        cout << m_muscleTensionTwentySix[i];
	if (i < m_muscleTensionTwentySix.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionTwentySeven.size(); ++i) {
        cout << m_muscleTensionTwentySeven[i];
	if (i < m_muscleTensionTwentySeven.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionTwentyEight.size(); ++i) {
        cout << m_muscleTensionTwentyEight[i];
	if (i < m_muscleTensionTwentyEight.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionTwentyNine.size(); ++i) {
        cout << m_muscleTensionTwentyNine[i];
	if (i < m_muscleTensionTwentyNine.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionThirty.size(); ++i) {
        cout << m_muscleTensionThirty[i];
	if (i < m_muscleTensionThirty.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionThirtyOne.size(); ++i) {
        cout << m_muscleTensionThirtyOne[i];
	if (i < m_muscleTensionThirtyOne.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionThirtyTwo.size(); ++i) {
        cout << m_muscleTensionThirtyTwo[i];
	if (i < m_muscleTensionThirtyTwo.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionThirtyThree.size(); ++i) {
        cout << m_muscleTensionThirtyThree[i];
	if (i < m_muscleTensionThirtyThree.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionThirtyFour.size(); ++i) {
        cout << m_muscleTensionThirtyFour[i];
	if (i < m_muscleTensionThirtyFour.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionThirtyFive.size(); ++i) {
        cout << m_muscleTensionThirtyFive[i];
	if (i < m_muscleTensionThirtyFive.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionThirtySix.size(); ++i) {
        cout << m_muscleTensionThirtySix[i];
	if (i < m_muscleTensionThirtySix.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionThirtySeven.size(); ++i) {
        cout << m_muscleTensionThirtySeven[i];
	if (i < m_muscleTensionThirtySeven.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionThirtyEight.size(); ++i) {
        cout << m_muscleTensionThirtyEight[i];
	if (i < m_muscleTensionThirtyEight.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionThirtyNine.size(); ++i) {
        cout << m_muscleTensionThirtyNine[i];
	if (i < m_muscleTensionThirtyNine.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionForty.size(); ++i) {
        cout << m_muscleTensionForty[i];
	if (i < m_muscleTensionForty.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionFortyOne.size(); ++i) {
        cout << m_muscleTensionFortyOne[i];
	if (i < m_muscleTensionFortyOne.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionFortyTwo.size(); ++i) {
        cout << m_muscleTensionFortyTwo[i];
	if (i < m_muscleTensionFortyTwo.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionFortyThree.size(); ++i) {
        cout << m_muscleTensionFortyThree[i];
	if (i < m_muscleTensionFortyThree.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionFortyFour.size(); ++i) {
        cout << m_muscleTensionFortyFour[i];
	if (i < m_muscleTensionFortyFour.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionFortyFive.size(); ++i) {
        cout << m_muscleTensionFortyFive[i];
	if (i < m_muscleTensionFortyFive.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionFortySix.size(); ++i) {
        cout << m_muscleTensionFortySix[i];
	if (i < m_muscleTensionFortySix.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionFortySeven.size(); ++i) {
        cout << m_muscleTensionFortySeven[i];
	if (i < m_muscleTensionFortySeven.size() - 1){
 	    cout<< ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionFortyEight.size(); ++i) {
        cout << m_muscleTensionFortyEight[i];
	if (i < m_muscleTensionFortyEight.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionFortyNine.size(); ++i) {
        cout << m_muscleTensionFortyNine[i];
	if (i < m_muscleTensionFortyNine.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionFifty.size(); ++i) {
        cout << m_muscleTensionFifty[i];
	if (i < m_muscleTensionFifty.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleTensionFiftyOne.size(); ++i) { 
        cout << m_muscleTensionFiftyOne[i];
	if (i < m_muscleTensionFiftyOne.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    //Printing Lengths, to redirect to a .csv (for plotting):
    for (int i = 0; i < m_muscleLengthZero.size(); ++i) {
        cout << m_muscleLengthZero[i];
	if (i < m_muscleLengthZero.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthOne.size(); ++i) {
        cout << m_muscleLengthOne[i];
	if (i < m_muscleLengthOne.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthTwo.size(); ++i) {
        cout << m_muscleLengthTwo[i];
	if (i < m_muscleLengthTwo.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;	

    for (int i = 0; i < m_muscleLengthThree.size(); ++i) {
        cout << m_muscleLengthThree[i];
	if (i < m_muscleLengthThree.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthFour.size(); ++i) {
        cout << m_muscleLengthFour[i];
	if (i < m_muscleLengthFour.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthFive.size(); ++i) {
        cout << m_muscleLengthFive[i];
	if (i < m_muscleLengthFive.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthSix.size(); ++i) {
        cout << m_muscleLengthSix[i];
	if (i < m_muscleLengthSix.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthSeven.size(); ++i) {
        cout << m_muscleLengthSeven[i];
	if (i < m_muscleLengthSeven.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthEight.size(); ++i) {
        cout << m_muscleLengthEight[i];
	if (i < m_muscleLengthEight.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthNine.size(); ++i) {
        cout << m_muscleLengthNine[i];
	if (i < m_muscleLengthNine.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthTen.size(); ++i) {
        cout << m_muscleLengthTen[i];
	if (i < m_muscleLengthTen.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthEleven.size(); ++i) {
        cout << m_muscleLengthEleven[i];
	if (i < m_muscleLengthEleven.size() - 1){
 	    cout<< ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthTwelve.size(); ++i) {
        cout << m_muscleLengthTwelve[i];
	if (i < m_muscleLengthTwelve.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthThirteen.size(); ++i) {
        cout << m_muscleLengthThirteen[i];
	if (i < m_muscleLengthThirteen.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthFourteen.size(); ++i) {
        cout << m_muscleLengthFourteen[i];
	if (i < m_muscleLengthFourteen.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthFifteen.size(); ++i) {
        cout << m_muscleLengthFifteen[i];
	if (i < m_muscleLengthFifteen.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthSixteen.size(); ++i) {
        cout << m_muscleLengthSixteen[i];
	if (i < m_muscleLengthSixteen.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthSeventeen.size(); ++i) {
        cout << m_muscleLengthSeventeen[i];
	if (i < m_muscleLengthSeventeen.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthEighteen.size(); ++i) {
        cout << m_muscleLengthEighteen[i];
	if (i < m_muscleLengthEighteen.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthNineteen.size(); ++i) {
        cout << m_muscleLengthNineteen[i];
	if (i < m_muscleLengthNineteen.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthTwenty.size(); ++i) {
        cout << m_muscleLengthTwenty[i];
	if (i < m_muscleLengthTwenty.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthTwentyOne.size(); ++i) {
        cout << m_muscleLengthTwentyOne[i];
	if (i < m_muscleLengthTwentyOne.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthTwentyTwo.size(); ++i) {
        cout << m_muscleLengthTwentyTwo[i];
	if (i < m_muscleLengthTwentyTwo.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthTwentyThree.size(); ++i) {
        cout << m_muscleLengthTwentyThree[i];
	if (i < m_muscleLengthTwentyThree.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthTwentyFour.size(); ++i) {
        cout << m_muscleLengthTwentyFour[i];
	if (i < m_muscleLengthTwentyFour.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthTwentyFive.size(); ++i) {
        cout << m_muscleLengthTwentyFive[i];
	if (i < m_muscleLengthTwentyFive.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthTwentySix.size(); ++i) {
        cout << m_muscleLengthTwentySix[i];
	if (i < m_muscleLengthTwentySix.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthTwentySeven.size(); ++i) {
        cout << m_muscleLengthTwentySeven[i];
	if (i < m_muscleLengthTwentySeven.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthTwentyEight.size(); ++i) {
        cout << m_muscleLengthTwentyEight[i];
	if (i < m_muscleLengthTwentyEight.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthTwentyNine.size(); ++i) {
        cout << m_muscleLengthTwentyNine[i];
	if (i < m_muscleLengthTwentyNine.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthThirty.size(); ++i) {
        cout << m_muscleLengthThirty[i];
	if (i < m_muscleLengthThirty.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthThirtyOne.size(); ++i) {
        cout << m_muscleLengthThirtyOne[i];
	if (i < m_muscleLengthThirtyOne.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthThirtyTwo.size(); ++i) {
        cout << m_muscleLengthThirtyTwo[i];
	if (i < m_muscleLengthThirtyTwo.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthThirtyThree.size(); ++i) {
        cout << m_muscleLengthThirtyThree[i];
	if (i < m_muscleLengthThirtyThree.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthThirtyFour.size(); ++i) {
        cout << m_muscleLengthThirtyFour[i];
	if (i < m_muscleLengthThirtyFour.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthThirtyFive.size(); ++i) {
        cout << m_muscleLengthThirtyFive[i];
	if (i < m_muscleLengthThirtyFive.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthThirtySix.size(); ++i) {
        cout << m_muscleLengthThirtySix[i];
	if (i < m_muscleLengthThirtySix.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthThirtySeven.size(); ++i) {
        cout << m_muscleLengthThirtySeven[i];
	if (i < m_muscleLengthThirtySeven.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthThirtyEight.size(); ++i) {
        cout << m_muscleLengthThirtyEight[i];
	if (i < m_muscleLengthThirtyEight.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthThirtyNine.size(); ++i) {
        cout << m_muscleLengthThirtyNine[i];
	if (i < m_muscleLengthThirtyNine.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthForty.size(); ++i) {
        cout << m_muscleLengthForty[i];
	if (i < m_muscleLengthForty.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthFortyOne.size(); ++i) {
        cout << m_muscleLengthFortyOne[i];
	if (i < m_muscleLengthFortyOne.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthFortyTwo.size(); ++i) {
        cout << m_muscleLengthFortyTwo[i];
	if (i < m_muscleLengthFortyTwo.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthFortyThree.size(); ++i) {
        cout << m_muscleLengthFortyThree[i];
	if (i < m_muscleLengthFortyThree.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthFortyFour.size(); ++i) {
        cout << m_muscleLengthFortyFour[i];
	if (i < m_muscleLengthFortyFour.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthFortyFive.size(); ++i) {
        cout << m_muscleLengthFortyFive[i];
	if (i < m_muscleLengthFortyFive.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthFortySix.size(); ++i) {
        cout << m_muscleLengthFortySix[i];
	if (i < m_muscleLengthFortySix.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthFortySeven.size(); ++i) {
        cout << m_muscleLengthFortySeven[i];
	if (i < m_muscleLengthFortySeven.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthFortyEight.size(); ++i) {
        cout << m_muscleLengthFortyEight[i];
	if (i < m_muscleLengthFortyEight.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthFortyNine.size(); ++i) {
        cout << m_muscleLengthFortyNine[i];
	if (i < m_muscleLengthFortyNine.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthFifty.size(); ++i) {
        cout << m_muscleLengthFifty[i];
	if (i < m_muscleLengthFifty.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i = 0; i < m_muscleLengthFiftyOne.size(); ++i) {
        cout << m_muscleLengthFiftyOne[i];
	if (i < m_muscleLengthFiftyOne.size() - 1){
 	    cout << ',';
	}
    }
    cout << endl;

    for (int i=0; i < m_quadCOM.size(); i++) {
        cout << m_quadCOM[i][0] << ',' << m_quadCOM[i][1] << ',' << m_quadCOM[i][2] << ',' << endl;
   }
   cout << endl;
    
}
