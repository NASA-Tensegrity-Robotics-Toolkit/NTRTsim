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
 * @file JSONAOHierarchyControl.cpp
 * @brief A controller for the template class BaseQuadModelLearning. 
 * @Includes A hierarchy of CPGs for MountainGoat
 * @author Dawn Hustig-Schultz, Brandon Gigous
 * @version 1.1.0
 * $Id$
 */

#include "JSONAOHierarchyControl.h"


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
//#define USE_KINEMATIC
//#define PRINT_METRICS

using namespace std;

JSONAOHierarchyControl::Config::Config(int ss,
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
					/*int ohm,
					int thm,
          				int olm,
          				int tlm, 
                                        int oam,
                                        int tam,*/
					int ohighm,
					int thighm,
					double hf2,
					double ffMax2) :
JSONQuadCPGControl::Config::Config(ss, tm, om, param, segnum, ct, la, ha,
                                    lp, hp, kt, kp, kv, def, cl, lf, hf),
freqFeedbackMin(ffMin),
freqFeedbackMax(ffMax),
ampFeedbackMin(afMin),
ampFeedbackMax(afMax),
phaseFeedbackMin(pfMin),
phaseFeedbackMax(pfMax),
maxHeight(maxH),
minHeight(minH),
/*ourHipMuscles(ohm),
theirHipMuscles(thm),
ourLegMuscles(olm),
theirLegMuscles(tlm),
ourAchillesMuscles(oam),
theirAchillesMuscles(tam),
ourHighMuscles(ohighm),
theirHighMuscles(thighm),*/
highFreq2(hf2),
freqFeedbackMax2(ffMax2)
{
    
}
/**
 * Defining the adapters here assumes the controller is around and
 * attached for the lifecycle of the learning runs. I.E. that the setup
 * and teardown functions are used for tgModel
 */
JSONAOHierarchyControl::JSONAOHierarchyControl(JSONAOHierarchyControl::Config config,	
                                                std::string args,
                                                std::string resourcePath) :
JSONQuadCPGControl(config, args, resourcePath),
m_config(config)
{
    // Path and filename handled by base class
    
}

JSONAOHierarchyControl::~JSONAOHierarchyControl()
{
    //delete nn;
}

void JSONAOHierarchyControl::onSetup(BaseQuadModelLearning& subject)
{
    m_pCPGSys = new CPGEquationsFB(500);

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
    // Lower level CPG node and edge params:
    /*Json::Value spineNodeVals = root.get("spineNodeVals", "UTF-8");
    Json::Value legNodeVals = root.get("legNodeVals", "UTF-8");
    Json::Value spineEdgeVals = root.get("spineEdgeVals", "UTF-8");
    Json::Value hipEdgeVals = root.get("hipEdgeVals", "UTF-8");
    Json::Value legEdgeVals = root.get("legEdgeVals", "UTF-8");*/
    Json::Value achillesNodeVals = root.get("achillesNodeVals", "UTF-8");
    Json::Value achillesEdgeVals = root.get("achillesEdgeVals", "UTF-8");    

    std::cout << achillesNodeVals << std::endl;
    
    /*spineNodeVals = spineNodeVals.get("params", "UTF-8");
    legNodeVals = legNodeVals.get("params", "UTF-8");
    spineEdgeVals = spineEdgeVals.get("params", "UTF-8");
    hipEdgeVals = hipEdgeVals.get("params", "UTF-8");
    legEdgeVals = legEdgeVals.get("params", "UTF-8");*/
    achillesNodeVals = achillesNodeVals.get("params", "UTF-8");
    achillesEdgeVals = achillesEdgeVals.get("params", "UTF-8");
    
    // A painful way of reducing the solution space... had to rewrite scaleEdgeActions() to take in a couple more parameters.
    //array_4D spineEdgeParams = scaleEdgeActions(spineEdgeVals,m_config.segmentSpan,m_config.theirMuscles,m_config.ourMuscles);
    //array_4D hipEdgeParams = scaleEdgeActions(hipEdgeVals,m_config.segmentSpan,m_config.theirHipMuscles,m_config.ourHipMuscles);
    //array_4D legEdgeParams = scaleEdgeActions(legEdgeVals,m_config.segmentSpan,m_config.theirLegMuscles,m_config.ourLegMuscles);
    array_4D achillesEdgeParams = scaleEdgeActions(achillesEdgeVals,m_config.segmentSpan,m_config.theirMuscles,m_config.ourMuscles);
    //array_2D spineNodeParams = scaleNodeActions(spineNodeVals, m_config.highFreq, m_config.freqFeedbackMax);
    //array_2D legNodeParams = scaleNodeActions(legNodeVals, m_config.highFreq, m_config.freqFeedbackMax);
    array_2D achillesNodeParams = scaleNodeActions(achillesNodeVals, m_config.highFreq, m_config.freqFeedbackMax);

    // Setup the lower level of CPGs
    setupCPGs(subject, achillesNodeParams, achillesEdgeParams);

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

    
#if(1)
    Json::Value PVal = root.get("propVals", "UTF-8");
    Json::Value DVal = root.get("derVals", "UTF-8");

	cout << PVal << endl;
    
	// Keep drilling if necessary
    PVal = PVal.get("params", "UTF-8");
    DVal = DVal.get("params", "UTF-8");

	if (PVal[0].isArray())
	{
		PVal = PVal[0];
	}
	if (DVal[0].isArray())
	{
		DVal = DVal[0];
	}
    
	int j = 0;
	P = (PVal.get(j, 0.0)).asDouble();
	D = (DVal.get(j, 0.0)).asDouble();
#endif

}

void JSONAOHierarchyControl::onStep(BaseQuadModelLearning& subject, double dt)
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
    double currentHeightRear =  subject.getSegmentCOM(6)[1];
    
    /// Max and min heights added to config
    if (currentHeight > m_config.maxHeight || currentHeight < m_config.minHeight || currentHeightRear > m_config.maxHeight || currentHeightRear < m_config.minHeight)
    {
		/// @todo if bogus, stop trial (reset simulation)
		bogus = true;
		throw std::runtime_error("Height out of range");
    }

}

void JSONAOHierarchyControl::onTeardown(BaseQuadModelLearning& subject)
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
    
    std::vector<tgSpringCableActuator* > tmpStrings = subject.find<tgSpringCableActuator> ("only ");
    
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
    
    delete m_pCPGSys;
    m_pCPGSys = NULL;
    
    // This is ugly, will clean up later:

    //Achilles controllers
    for(size_t i = 0; i < m_leftFrontAchillesControllers.size(); i++)
    {
        delete m_leftFrontAchillesControllers[i];
    }
    m_leftFrontAchillesControllers.clear();  

    for(size_t i = 0; i < m_rightFrontAchillesControllers.size(); i++)
    {
        delete m_rightFrontAchillesControllers[i];
    }
    m_rightFrontAchillesControllers.clear(); 

    for(size_t i = 0; i < m_leftRearAchillesControllers.size(); i++)
    {
        delete m_leftRearAchillesControllers[i];
    }
    m_leftRearAchillesControllers.clear();  

    for(size_t i = 0; i < m_rightRearAchillesControllers.size(); i++)
    {
        delete m_rightRearAchillesControllers[i];
    }
    m_rightRearAchillesControllers.clear();  

    // Trying to delete here instead, to fix the leak
    delete nn;

}

//Note: Will make a more compact, reuseable function later when I decide on how want to handle m_xControllers for different body parts
//Perhaps a vector of vectors can be created, so looping can be done instead, or the function can be reused. 
//For now, I'll settle for horrendous but works. 
//This way makes it easier to grab the right node numbers again when doing high-low couplings.
void JSONAOHierarchyControl::setupCPGs(BaseQuadModelLearning& subject, array_2D achillesNodeActions, array_4D achillesEdgeActions)
{
	    
    // Achilles muscles
    std::vector <tgSpringCableActuator*> leftFrontAchillesMuscles = subject.find<tgSpringCableActuator> ("left_front_achilles_tendon");
    std::vector <tgSpringCableActuator*> rightFrontAchillesMuscles = subject.find<tgSpringCableActuator> ("right_front_achilles_tendon");

    std::vector <tgSpringCableActuator*> leftRearAchillesMuscles = subject.find<tgSpringCableActuator> ("left_rear_achilles_tendon");
    std::vector <tgSpringCableActuator*> rightRearAchillesMuscles = subject.find<tgSpringCableActuator> ("right_rear_achilles_tendon");
    
    CPGEquationsFB& m_CPGFBSys = *(tgCast::cast<CPGEquations, CPGEquationsFB>(m_pCPGSys));


    
    /*
     * ACHILLES START HERE
     */
    // Left front achilles first
    for (std::size_t i = 0; i < leftFrontAchillesMuscles.size(); i++)
    {

        tgPIDController::Config config(P, 0.0, D, true); // Non backdrivable
        tgCPGCableControl* pStringControl = new tgCPGCableControl(config);

        leftFrontAchillesMuscles[i]->attach(pStringControl);
        
        // First assign node numbers
        pStringControl->assignNodeNumberFB(m_CPGFBSys, achillesNodeActions);
        
        m_leftFrontAchillesControllers.push_back(pStringControl);
    }
	
    // Then determine connectivity and setup string
    for (std::size_t i = 0; i < m_leftFrontAchillesControllers.size(); i++)
    {
        tgCPGActuatorControl * const pStringInfo = m_leftFrontAchillesControllers[i];
        assert(pStringInfo != NULL);
        pStringInfo->setConnectivity(m_leftFrontAchillesControllers, achillesEdgeActions);
        
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

    // Now right front achilles
    for (std::size_t i = 0; i < rightFrontAchillesMuscles.size(); i++)
    {

        tgPIDController::Config config(P, 0.0, D, true); // Non backdrivable
        tgCPGCableControl* pStringControl = new tgCPGCableControl(config);

        rightFrontAchillesMuscles[i]->attach(pStringControl);
        
        // First assign node numbers
        pStringControl->assignNodeNumberFB(m_CPGFBSys, achillesNodeActions);
        
        m_rightFrontAchillesControllers.push_back(pStringControl);
    }
	
    // Then determine connectivity and setup string
    for (std::size_t i = 0; i < m_rightFrontAchillesControllers.size(); i++)
    {
        tgCPGActuatorControl * const pStringInfo = m_rightFrontAchillesControllers[i];
        assert(pStringInfo != NULL);
        pStringInfo->setConnectivity(m_rightFrontAchillesControllers, achillesEdgeActions);
        
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

    // Now left rear achilles
    for (std::size_t i = 0; i < leftRearAchillesMuscles.size(); i++)
    {

        tgPIDController::Config config(P, 0.0, D, true); // Non backdrivable
        tgCPGCableControl* pStringControl = new tgCPGCableControl(config);

        leftRearAchillesMuscles[i]->attach(pStringControl);
        
        // First assign node numbers
        pStringControl->assignNodeNumberFB(m_CPGFBSys, achillesNodeActions);
        
        m_leftRearAchillesControllers.push_back(pStringControl);
    }
	
    // Then determine connectivity and setup string
    for (std::size_t i = 0; i < m_leftRearAchillesControllers.size(); i++)
    {
        tgCPGActuatorControl * const pStringInfo = m_leftRearAchillesControllers[i];
        assert(pStringInfo != NULL);
        pStringInfo->setConnectivity(m_leftRearAchillesControllers, achillesEdgeActions);
        
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

    // Finally, right rear achilles
    for (std::size_t i = 0; i < rightRearAchillesMuscles.size(); i++)
    {

        tgPIDController::Config config(P, 0.0, D, true); // Non backdrivable
        tgCPGCableControl* pStringControl = new tgCPGCableControl(config);

        rightRearAchillesMuscles[i]->attach(pStringControl);
        
        // First assign node numbers
        pStringControl->assignNodeNumberFB(m_CPGFBSys, achillesNodeActions);
        
        m_rightRearAchillesControllers.push_back(pStringControl);
    }
	
    // Then determine connectivity and setup string
    for (std::size_t i = 0; i < m_rightRearAchillesControllers.size(); i++)
    {
        tgCPGActuatorControl * const pStringInfo = m_rightRearAchillesControllers[i];
        assert(pStringInfo != NULL);
        pStringInfo->setConnectivity(m_rightRearAchillesControllers, achillesEdgeActions);
        
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

array_2D JSONAOHierarchyControl::scaleNodeActions (Json::Value actions, double highFreq, double freqFeedbackMax)
{
    std::size_t numControllers = actions.size();
    std::size_t numActions = actions[0].size();
    
    array_2D nodeActions(boost::extents[numControllers][numActions]);
    
    array_2D limits(boost::extents[2][numActions]);
    
    // Check if we need to update limits
    assert(numActions == 5);
    
	limits[0][0] = m_config.lowFreq;
	limits[1][0] = highFreq;
	limits[0][1] = m_config.lowAmp;
	limits[1][1] = m_config.highAmp;
    limits[0][2] = m_config.freqFeedbackMin;
    limits[1][2] = freqFeedbackMax;
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

array_4D JSONAOHierarchyControl::scaleEdgeActions  
                            (Json::Value edgeParam, int segmentSpan, int theirMuscles, int ourMuscles)
{
    assert(edgeParam[0].size() == 2);
    
    double lowerLimit = m_config.lowPhase;
    double upperLimit = m_config.highPhase;
    double range = upperLimit - lowerLimit;
    
    array_4D actionList(boost::extents[segmentSpan][theirMuscles][ourMuscles][m_config.params]);
    
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
    
    while (i < segmentSpan)
    {
        while(j < theirMuscles)
        {
            while(k < ourMuscles)
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

std::vector<double> JSONAOHierarchyControl::getFeedback(BaseQuadModelLearning& subject)
{
    // Placeholder
    std::vector<double> feedback;
    
    const std::vector<tgSpringCableActuator*>& allCables = subject.find<tgSpringCableActuator> ("only ");
    
    double *inputs = new double[m_config.numStates];
    
    std::size_t n = allCables.size();
    for(std::size_t i = 0; i != n; i++)
    {
        std::vector< std::vector<double> > actions;
        
        const tgSpringCableActuator& cable = *(allCables[i]);
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

std::vector<double> JSONAOHierarchyControl::getCableState(const tgSpringCableActuator& cable)
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

std::vector<double> JSONAOHierarchyControl::transformFeedbackActions(std::vector< std::vector<double> >& actions)
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
