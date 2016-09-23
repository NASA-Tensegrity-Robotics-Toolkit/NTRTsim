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
 * @file JSONHierarchyFeedbackControl.cpp
 * @brief A controller for the template class BaseSpineModelLearning. 
 * @Includes A hierarchy of CPGs for MountainGoat
 * @author Dawn Hustig-Schultz, Brandon Gigous
 * @version 1.1.0
 * $Id$
 */

#include "JSONHierarchyFeedbackControl.h"


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
//#define PRINT_METRICS

using namespace std;

JSONHierarchyFeedbackControl::Config::Config(int ss,
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
					int ohm,
					int thm,
          				int olm,
          				int tlm, 
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
ourHipMuscles(ohm),
theirHipMuscles(thm),
ourLegMuscles(olm),
theirLegMuscles(tlm),
ourHighMuscles(ohighm),
theirHighMuscles(thighm),
highFreq2(hf2),
freqFeedbackMax2(ffMax2)
{
    
}
/**
 * Defining the adapters here assumes the controller is around and
 * attached for the lifecycle of the learning runs. I.E. that the setup
 * and teardown functions are used for tgModel
 */
JSONHierarchyFeedbackControl::JSONHierarchyFeedbackControl(JSONHierarchyFeedbackControl::Config config,	
                                                std::string args,
                                                std::string resourcePath) :
JSONQuadCPGControl(config, args, resourcePath),
m_config(config)
{
    // Path and filename handled by base class
    
}

JSONHierarchyFeedbackControl::~JSONHierarchyFeedbackControl()
{
    delete nn;
}

void JSONHierarchyFeedbackControl::onSetup(BaseQuadModelLearning& subject)
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
    // Lower level CPG node and edge params:
    Json::Value spineNodeVals = root.get("spineNodeVals", "UTF-8");
    Json::Value legNodeVals = root.get("legNodeVals", "UTF-8");
    Json::Value spineEdgeVals = root.get("spineEdgeVals", "UTF-8");
    Json::Value hipEdgeVals = root.get("hipEdgeVals", "UTF-8");
    Json::Value legEdgeVals = root.get("legEdgeVals", "UTF-8");

    std::cout << spineNodeVals << std::endl;
    
    spineNodeVals = spineNodeVals.get("params", "UTF-8");
    legNodeVals = legNodeVals.get("params", "UTF-8");
    spineEdgeVals = spineEdgeVals.get("params", "UTF-8");
    hipEdgeVals = hipEdgeVals.get("params", "UTF-8");
    legEdgeVals = legEdgeVals.get("params", "UTF-8");
    
    // A painful way of reducing the solution space... had to rewrite scaleEdgeActions() to take in a couple more parameters.
    array_4D spineEdgeParams = scaleEdgeActions(spineEdgeVals,m_config.segmentSpan,m_config.theirMuscles,m_config.ourMuscles);
    array_4D hipEdgeParams = scaleEdgeActions(hipEdgeVals,m_config.segmentSpan,m_config.theirHipMuscles,m_config.ourHipMuscles);
    array_4D legEdgeParams = scaleEdgeActions(legEdgeVals,m_config.segmentSpan,m_config.theirLegMuscles,m_config.ourLegMuscles);
    array_2D spineNodeParams = scaleNodeActions(spineNodeVals, m_config.highFreq, m_config.freqFeedbackMax);
    array_2D legNodeParams = scaleNodeActions(legNodeVals, m_config.highFreq, m_config.freqFeedbackMax);


    // Higher level CPG node and edge params:
    Json::Value highNodeVals = root.get("highNodeVals", "UTF-8");
    Json::Value highEdgeVals = root.get("highEdgeVals", "UTF-8");

    highNodeVals = highNodeVals.get("params", "UTF-8");
    highEdgeVals = highEdgeVals.get("params", "UTF-8");

    // Edge params for couplings between higher and lower levels:
    Json::Value highLowEdgeVals = root.get("hLowVals", "UTF-8");
    highLowEdgeVals = highLowEdgeVals.get("params", "UTF-8");

    array_4D highEdgeParams = scaleEdgeActions(highEdgeVals,2,m_config.theirHighMuscles,m_config.ourHighMuscles); 
    array_2D highNodeParams = scaleNodeActions(highNodeVals, m_config.highFreq2, m_config.freqFeedbackMax2);

    // Setup the higher level of CPGs
    setupHighCPGs(subject, highNodeParams, highEdgeParams);

    // Setup the lower level of CPGs
    setupCPGs(subject, spineNodeParams, legNodeParams, spineEdgeParams, hipEdgeParams, legEdgeParams);

    // Setup the couplings between higher and lower level CPGs
    setupHighLowCouplings(subject, highLowEdgeVals);

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

void JSONHierarchyFeedbackControl::onStep(BaseQuadModelLearning& subject, double dt)
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

}

void JSONHierarchyFeedbackControl::onTeardown(BaseQuadModelLearning& subject)
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
    
    std::vector<tgSpringCableActuator* > tmpStrings = subject.find<tgSpringCableActuator> ("all ");
    
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
    
    for(size_t i = 0; i < m_spineControllers.size(); i++)
    {
        delete m_spineControllers[i];
    }
    m_spineControllers.clear();    

    for(size_t i = 0; i < m_leftShoulderControllers.size(); i++)
    {
        delete m_leftShoulderControllers[i];
    }
    m_leftShoulderControllers.clear(); 

    for(size_t i = 0; i < m_rightShoulderControllers.size(); i++)
    {
        delete m_rightShoulderControllers[i];
    }
    m_rightShoulderControllers.clear(); 

    for(size_t i = 0; i < m_leftHipControllers.size(); i++)
    {
        delete m_leftHipControllers[i];
    }
    m_leftHipControllers.clear(); 

    for(size_t i = 0; i < m_rightHipControllers.size(); i++)
    {
        delete m_rightHipControllers[i];
    }
    m_rightHipControllers.clear(); 

    for(size_t i = 0; i < m_leftForelegControllers.size(); i++)
    {
        delete m_leftForelegControllers[i];
    }
    m_leftForelegControllers.clear();  

    for(size_t i = 0; i < m_rightForelegControllers.size(); i++)
    {
        delete m_rightForelegControllers[i];
    }
    m_rightForelegControllers.clear(); 

    for(size_t i = 0; i < m_leftHindlegControllers.size(); i++)
    {
        delete m_leftHindlegControllers[i];
    }
    m_leftHindlegControllers.clear();  

    for(size_t i = 0; i < m_rightHindlegControllers.size(); i++)
    {
        delete m_rightHindlegControllers[i];
    }
    m_rightHindlegControllers.clear();  

    for(size_t i = 0; i < m_highControllers.size(); i++)
    {
        delete m_highControllers[i];
    }
    m_highControllers.clear();  

}

//Note: Will make a more compact, reuseable function later when I decide on how want to handle m_xControllers for different body parts
//Perhaps a vector of vectors can be created, so looping can be done instead, or the function can be reused. 
//For now, I'll settle for horrendous but works. 
//This way makes it easier to grab the right node numbers again when doing high-low couplings.
void JSONHierarchyFeedbackControl::setupCPGs(BaseQuadModelLearning& subject, array_2D spineNodeActions, array_2D legNodeActions, array_4D spineEdgeActions, array_4D hipEdgeActions, array_4D legEdgeActions)
{
	    
    std::vector <tgSpringCableActuator*> spineMuscles = subject.find<tgSpringCableActuator> ("spine ");

    std::vector <tgSpringCableActuator*> leftShoulderMuscles = subject.find<tgSpringCableActuator> ("left_shoulder");
    std::vector <tgSpringCableActuator*> rightShoulderMuscles = subject.find<tgSpringCableActuator> ("right_shoulder");  

    std::vector <tgSpringCableActuator*> leftHipMuscles = subject.find<tgSpringCableActuator> ("left_hip ");
    std::vector <tgSpringCableActuator*> rightHipMuscles = subject.find<tgSpringCableActuator> ("right_hip ");

    std::vector <tgSpringCableActuator*> leftForelegMuscles = subject.find<tgSpringCableActuator> ("left_foreleg");
    std::vector <tgSpringCableActuator*> rightForelegMuscles = subject.find<tgSpringCableActuator> ("right_foreleg");

    std::vector <tgSpringCableActuator*> leftHindlegMuscles = subject.find<tgSpringCableActuator> ("left_hindleg");
    std::vector <tgSpringCableActuator*> rightHindlegMuscles = subject.find<tgSpringCableActuator> ("right_hindleg");
    
    CPGEquationsFB& m_CPGFBSys = *(tgCast::cast<CPGEquations, CPGEquationsFB>(m_pCPGSys));

    /*
     * SPINE COUPLING STARTS HERE
     */
    
    for (std::size_t i = 0; i < spineMuscles.size(); i++)
    {

        tgPIDController::Config config(20000.0, 0.0, 5.0, true); // Non backdrivable
        tgCPGCableControl* pStringControl = new tgCPGCableControl(config);

        spineMuscles[i]->attach(pStringControl);
        
        // First assign node numbers
        pStringControl->assignNodeNumberFB(m_CPGFBSys, spineNodeActions);
        
        m_spineControllers.push_back(pStringControl);
    }
    
    // Then determine connectivity and setup string
    for (std::size_t i = 0; i < m_spineControllers.size(); i++)
    {
        tgCPGActuatorControl * const pStringInfo = m_spineControllers[i];
        assert(pStringInfo != NULL);
        pStringInfo->setConnectivity(m_spineControllers, spineEdgeActions);
        
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

    /*
     * HIPS START HERE
     */

    //Left shoulder first.
    for (std::size_t i = 0; i < leftShoulderMuscles.size(); i++)
    {

        tgPIDController::Config config(20000.0, 0.0, 5.0, true); // Non backdrivable
        tgCPGCableControl* pStringControl = new tgCPGCableControl(config);

        leftShoulderMuscles[i]->attach(pStringControl);
        
        // First assign node numbers
        pStringControl->assignNodeNumberFB(m_CPGFBSys, legNodeActions);
        
        m_leftShoulderControllers.push_back(pStringControl);
    }
    
    // Then determine connectivity and setup string
    for (std::size_t i = 0; i < m_leftShoulderControllers.size(); i++)
    {
        tgCPGActuatorControl * const pStringInfo = m_leftShoulderControllers[i];
        assert(pStringInfo != NULL);
        pStringInfo->setConnectivity(m_leftShoulderControllers, hipEdgeActions);
        
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

    //Now right shoulder.
    for (std::size_t i = 0; i < rightShoulderMuscles.size(); i++)
    {

        tgPIDController::Config config(20000.0, 0.0, 5.0, true); // Non backdrivable
        tgCPGCableControl* pStringControl = new tgCPGCableControl(config);

        rightShoulderMuscles[i]->attach(pStringControl);
        
        // First assign node numbers
        pStringControl->assignNodeNumberFB(m_CPGFBSys, legNodeActions);
        
        m_rightShoulderControllers.push_back(pStringControl);
    }
    
    // Then determine connectivity and setup string
    for (std::size_t i = 0; i < m_rightShoulderControllers.size(); i++)
    {
        tgCPGActuatorControl * const pStringInfo = m_rightShoulderControllers[i];
        assert(pStringInfo != NULL);
        pStringInfo->setConnectivity(m_rightShoulderControllers, hipEdgeActions);
        
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

    // Now the left hip
    for (std::size_t i = 0; i < leftHipMuscles.size(); i++)
    {

        tgPIDController::Config config(20000.0, 0.0, 5.0, true); // Non backdrivable
        tgCPGCableControl* pStringControl = new tgCPGCableControl(config);

        leftHipMuscles[i]->attach(pStringControl);
        
        // First assign node numbers
        pStringControl->assignNodeNumberFB(m_CPGFBSys, legNodeActions);
        
        m_leftHipControllers.push_back(pStringControl);
    }
    
    // Then determine connectivity and setup string
    for (std::size_t i = 0; i < m_leftHipControllers.size(); i++)
    {
        tgCPGActuatorControl * const pStringInfo = m_leftHipControllers[i];
        assert(pStringInfo != NULL);
        pStringInfo->setConnectivity(m_leftHipControllers, hipEdgeActions);
        
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

    // Now the right hip
    for (std::size_t i = 0; i < rightHipMuscles.size(); i++)
    {

        tgPIDController::Config config(20000.0, 0.0, 5.0, true); // Non backdrivable
        tgCPGCableControl* pStringControl = new tgCPGCableControl(config);

        leftHipMuscles[i]->attach(pStringControl);
        
        // First assign node numbers
        pStringControl->assignNodeNumberFB(m_CPGFBSys, legNodeActions);
        
        m_rightHipControllers.push_back(pStringControl);
    }
    
    // Then determine connectivity and setup string
    for (std::size_t i = 0; i < m_rightHipControllers.size(); i++)
    {
        tgCPGActuatorControl * const pStringInfo = m_rightHipControllers[i];
        assert(pStringInfo != NULL);
        pStringInfo->setConnectivity(m_rightHipControllers, hipEdgeActions);
        
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

    /*
     * LEGS START HERE
     */

    // Left foreleg first
    for (std::size_t i = 0; i < leftForelegMuscles.size(); i++)
    {

        tgPIDController::Config config(20000.0, 0.0, 5.0, true); // Non backdrivable
        tgCPGCableControl* pStringControl = new tgCPGCableControl(config);

        leftForelegMuscles[i]->attach(pStringControl);
        
        // First assign node numbers
        pStringControl->assignNodeNumberFB(m_CPGFBSys, legNodeActions);
        
        m_leftForelegControllers.push_back(pStringControl);
    }
	
    // Then determine connectivity and setup string
    for (std::size_t i = 0; i < m_leftForelegControllers.size(); i++)
    {
        tgCPGActuatorControl * const pStringInfo = m_leftForelegControllers[i];
        assert(pStringInfo != NULL);
        pStringInfo->setConnectivity(m_leftForelegControllers, legEdgeActions);
        
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

    // Now right foreleg
    for (std::size_t i = 0; i < rightForelegMuscles.size(); i++)
    {

        tgPIDController::Config config(20000.0, 0.0, 5.0, true); // Non backdrivable
        tgCPGCableControl* pStringControl = new tgCPGCableControl(config);

        rightForelegMuscles[i]->attach(pStringControl);
        
        // First assign node numbers
        pStringControl->assignNodeNumberFB(m_CPGFBSys, legNodeActions);
        
        m_rightForelegControllers.push_back(pStringControl);
    }
	
    // Then determine connectivity and setup string
    for (std::size_t i = 0; i < m_rightForelegControllers.size(); i++)
    {
        tgCPGActuatorControl * const pStringInfo = m_rightForelegControllers[i];
        assert(pStringInfo != NULL);
        pStringInfo->setConnectivity(m_rightForelegControllers, legEdgeActions);
        
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

    // Now left hindleg
    for (std::size_t i = 0; i < leftHindlegMuscles.size(); i++)
    {

        tgPIDController::Config config(20000.0, 0.0, 5.0, true); // Non backdrivable
        tgCPGCableControl* pStringControl = new tgCPGCableControl(config);

        leftHindlegMuscles[i]->attach(pStringControl);
        
        // First assign node numbers
        pStringControl->assignNodeNumberFB(m_CPGFBSys, legNodeActions);
        
        m_leftHindlegControllers.push_back(pStringControl);
    }
	
    // Then determine connectivity and setup string
    for (std::size_t i = 0; i < m_leftHindlegControllers.size(); i++)
    {
        tgCPGActuatorControl * const pStringInfo = m_leftHindlegControllers[i];
        assert(pStringInfo != NULL);
        pStringInfo->setConnectivity(m_leftHindlegControllers, legEdgeActions);
        
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

    // Finally, right hindleg
    for (std::size_t i = 0; i < rightHindlegMuscles.size(); i++)
    {

        tgPIDController::Config config(20000.0, 0.0, 5.0, true); // Non backdrivable
        tgCPGCableControl* pStringControl = new tgCPGCableControl(config);

        rightHindlegMuscles[i]->attach(pStringControl);
        
        // First assign node numbers
        pStringControl->assignNodeNumberFB(m_CPGFBSys, legNodeActions);
        
        m_rightHindlegControllers.push_back(pStringControl);
    }
	
    // Then determine connectivity and setup string
    for (std::size_t i = 0; i < m_rightHindlegControllers.size(); i++)
    {
        tgCPGActuatorControl * const pStringInfo = m_rightHindlegControllers[i];
        assert(pStringInfo != NULL);
        pStringInfo->setConnectivity(m_rightHindlegControllers, legEdgeActions);
        
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

void JSONHierarchyFeedbackControl::setupHighLowCouplings(BaseQuadModelLearning& subject, Json::Value highLowEdgeActions)
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

	//Unrolling loop for now... again, horrendous, and will think of a way to roll it back up eventually
	//First high CPG, b == 0:
	int b = 0;
	edgeIt--;

	std::vector<int> spineConnectivityList;
	std::vector<double> spineWeights;
	std::vector<double> spinePhases;

	std::vector<int> leftShoulderConnectivityList;
	std::vector<double> leftShoulderWeights;
	std::vector<double> leftShoulderPhases;

	std::vector<int> leftForelegConnectivityList;
	std::vector<double> leftForelegWeights;
	std::vector<double> leftForelegPhases;

	std::vector<int> rightShoulderConnectivityList;
	std::vector<double> rightShoulderWeights;
	std::vector<double> rightShoulderPhases;

	std::vector<int> rightForelegConnectivityList;
	std::vector<double> rightForelegWeights;
	std::vector<double> rightForelegPhases;

	std::vector<int> leftHipConnectivityList;
	std::vector<double> leftHipWeights;
	std::vector<double> leftHipPhases;

	std::vector<int> leftHindlegConnectivityList;
	std::vector<double> leftHindlegWeights;
	std::vector<double> leftHindlegPhases;

	std::vector<int> rightHipConnectivityList;
	std::vector<double> rightHipWeights;
	std::vector<double> rightHipPhases;

	std::vector<int> rightHindlegConnectivityList;
	std::vector<double> rightHindlegWeights;
	std::vector<double> rightHindlegPhases;
	
	Json::Value param = *edgeIt;	
	assert(param.size() == 2);

	// Use the vector of "m_controllers" vector for the part in question, to find the right node numbers
														
	for (size_t i = 0; i < m_spineControllers.size(); i++)
	{
		// Get the node number for each muscle 
		int nodeNum = m_spineControllers[i]->getNodeNumber();

		// And then add to the connectivity list. 
		spineConnectivityList.push_back(nodeNum);
		spineWeights.push_back(param[0].asDouble());
		spinePhases.push_back(param[1].asDouble() * (range) + lowerLimit);
	}
	m_CPGFBSys.defineConnections(b, spineConnectivityList, spineWeights, spinePhases);

	b++;
	edgeIt--;

	for (size_t i = 0; i < m_leftShoulderControllers.size(); i++)
	{
		// Get the node number for each muscle 
		int nodeNum = m_leftShoulderControllers[i]->getNodeNumber();

		// And then add to the connectivity list. 
		leftShoulderConnectivityList.push_back(nodeNum);
		leftShoulderWeights.push_back(param[0].asDouble());
		leftShoulderPhases.push_back(param[1].asDouble() * (range) + lowerLimit);
	}
	m_CPGFBSys.defineConnections(b, leftShoulderConnectivityList, leftShoulderWeights, leftShoulderPhases);

	//Do not increment/decrement b/edgeIt again, until full left foreleg leg is done!

	for (size_t i = 0; i < m_leftForelegControllers.size(); i++)
	{
		// Get the node number for each muscle 
		int nodeNum = m_leftForelegControllers[i]->getNodeNumber();

		// And then add to the connectivity list. 
		leftForelegConnectivityList.push_back(nodeNum);
		leftForelegWeights.push_back(param[0].asDouble());
		leftForelegPhases.push_back(param[1].asDouble() * (range) + lowerLimit);
	}
	m_CPGFBSys.defineConnections(b, leftForelegConnectivityList, leftForelegWeights, leftForelegPhases);

	// Now it's safe to increment/decrement

	b++;
	edgeIt--;

	for (size_t i = 0; i < m_rightShoulderControllers.size(); i++)
	{
		// Get the node number for each muscle 
		int nodeNum = m_rightShoulderControllers[i]->getNodeNumber();

		// And then add to the connectivity list. 
		rightShoulderConnectivityList.push_back(nodeNum);
		rightShoulderWeights.push_back(param[0].asDouble());
		rightShoulderPhases.push_back(param[1].asDouble() * (range) + lowerLimit);
	}
	m_CPGFBSys.defineConnections(b, rightShoulderConnectivityList, rightShoulderWeights, rightShoulderPhases);

	//Again, do not increment/decrement b/edgeIt until full right foreleg leg is done!
	
	for (size_t i = 0; i < m_rightForelegControllers.size(); i++)
	{
		// Get the node number for each muscle 
		int nodeNum = m_rightForelegControllers[i]->getNodeNumber();

		// And then add to the connectivity list. 
		rightForelegConnectivityList.push_back(nodeNum);
		rightForelegWeights.push_back(param[0].asDouble());
		rightForelegPhases.push_back(param[1].asDouble() * (range) + lowerLimit);
	}
	m_CPGFBSys.defineConnections(b, rightForelegConnectivityList, rightForelegWeights, rightForelegPhases);

	// Now it's safe to increment/decrement

	b++;
	edgeIt--;
	
	for (size_t i = 0; i < m_leftHipControllers.size(); i++)
	{
		// Get the node number for each muscle 
		int nodeNum = m_leftHipControllers[i]->getNodeNumber();

		// And then add to the connectivity list. 
		leftHipConnectivityList.push_back(nodeNum);
		leftHipWeights.push_back(param[0].asDouble());
		leftHipPhases.push_back(param[1].asDouble() * (range) + lowerLimit);
	}
	m_CPGFBSys.defineConnections(b, leftHipConnectivityList, leftHipWeights, leftHipPhases);

	//Do not increment/decrement b/edgeIt again, until full left hindleg leg is done!

	for (size_t i = 0; i < m_leftHindlegControllers.size(); i++)
	{
		// Get the node number for each muscle 
		int nodeNum = m_leftHindlegControllers[i]->getNodeNumber();

		// And then add to the connectivity list. 
		leftHindlegConnectivityList.push_back(nodeNum);
		leftHindlegWeights.push_back(param[0].asDouble());
		leftHindlegPhases.push_back(param[1].asDouble() * (range) + lowerLimit);
	}
	m_CPGFBSys.defineConnections(b, leftHindlegConnectivityList, leftHindlegWeights, leftHindlegPhases);

	// Now it's safe to increment/decrement

	b++;
	edgeIt--;

	for (size_t i = 0; i < m_rightHipControllers.size(); i++)
	{
		// Get the node number for each muscle 
		int nodeNum = m_rightHipControllers[i]->getNodeNumber();

		// And then add to the connectivity list. 
		rightHipConnectivityList.push_back(nodeNum);
		rightHipWeights.push_back(param[0].asDouble());
		rightHipPhases.push_back(param[1].asDouble() * (range) + lowerLimit);
	}
	m_CPGFBSys.defineConnections(b, rightHipConnectivityList, rightHipWeights, rightHipPhases);

	//Again, do not increment/decrement b/edgeIt until full right foreleg leg is done!

	
	for (size_t i = 0; i < m_rightHindlegControllers.size(); i++)
	{
		// Get the node number for each muscle 
		int nodeNum = m_rightHindlegControllers[i]->getNodeNumber();

		// And then add to the connectivity list. 
		rightHindlegConnectivityList.push_back(nodeNum);
		rightHindlegWeights.push_back(param[0].asDouble());
		rightHindlegPhases.push_back(param[1].asDouble() * (range) + lowerLimit);
	}
	m_CPGFBSys.defineConnections(b, rightHindlegConnectivityList, rightHindlegWeights, rightHindlegPhases);

	// TODO?
	assert(highLowEdgeActions.begin() == edgeIt);
}

void JSONHierarchyFeedbackControl::setupHighCPGs(BaseQuadModelLearning& subject, array_2D highNodeActions, array_4D highEdgeActions)
{
    CPGEquationsFB& m_CPGFBSys = *(tgCast::cast<CPGEquations, CPGEquationsFB>(m_pCPGSys));

	// Make higher level CPGs, give them node params
	for (std::size_t b = 0; b < 5; b++)
	{
        tgPIDController::Config config(20000.0, 0.0, 5.0, true); // Non backdrivable
        tgCPGCableControl* pStringControl = new tgCPGCableControl(config);

		// need to make a 1 row 2D array so we can pass it into assignNodeNumber.
		// 	using nodeActions will not work. (notice subtle name difference) ~B
		//array_2D nodeAction(boost::extents[1][nodeActions.shape()[1]]);
		//for (std::size_t a = 0; a < nodeActions.shape()[1]; a++)
			//nodeAction[0][a] = nodeActions[b][a];

		// must give node number, to make things easier and stuff
		pStringControl->assignNodeNumberFB(m_CPGFBSys, highNodeActions);

		m_highControllers.push_back(pStringControl);
	}

	// Determine connectivity between higher level CPGs
	for (std::size_t i = 0; i < m_highControllers.size(); i++)
	{
		tgCPGActuatorControl * const pStringInfo = m_highControllers[i];
		assert(pStringInfo != NULL);
		pStringInfo->setConnectivity(m_highControllers, highEdgeActions);
	}

}

array_2D JSONHierarchyFeedbackControl::scaleNodeActions (Json::Value actions, double highFreq, double freqFeedbackMax)
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

array_4D JSONHierarchyFeedbackControl::scaleEdgeActions  
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

std::vector<double> JSONHierarchyFeedbackControl::getFeedback(BaseQuadModelLearning& subject)
{
    // Placeholder
    std::vector<double> feedback;
    
    const std::vector<tgSpringCableActuator*>& allCables = subject.find<tgSpringCableActuator> ("all ");
    
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

    // inputting 0 for now
    std::size_t n2 = m_highControllers.size();
    for (size_t i = 0; i != n2; i++)
    {
	std::vector< std::vector<double> > actions;
		
	// Rescale to 0 to 1 (consider doing this inside getState)
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

std::vector<double> JSONHierarchyFeedbackControl::getCableState(const tgSpringCableActuator& cable)
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

std::vector<double> JSONHierarchyFeedbackControl::transformFeedbackActions(std::vector< std::vector<double> >& actions)
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
