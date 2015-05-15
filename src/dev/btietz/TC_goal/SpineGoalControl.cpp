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
 * @file SpineGoalControl.cpp
 * @brief A controller for the template class BaseSpineModelLearning
 * @author Brian Mirletz
 * @version 1.1.0
 * $Id$
 */

#include "SpineGoalControl.h"

#include "BaseSpineModelGoal.h"

// Should include tgString, but compiler complains since its been
// included from BaseSpineModelLearning. Perhaps we should move things
// to a cpp over there
#include "core/tgSpringCableActuator.h"
#include "core/tgBasicActuator.h"
#include "tgcreator/tgUtil.h"
#include "controllers/tgImpedanceController.h"
#include "examples/learningSpines/tgCPGActuatorControl.h"
#include "dev/CPG_feedback/tgCPGCableControl.h"

#include "helpers/FileHelpers.h"

#include "learning/AnnealEvolution/AnnealEvolution.h"
#include "learning/Configuration/configuration.h"

#include "dev/CPG_feedback/CPGEquationsFB.h"
#include "dev/CPG_feedback/CPGNodeFB.h"

#include <iterator>     // std::iterator
#include <string>

//#define LOGGING
#define USE_KINEMATIC

using namespace std;

SpineGoalControl::Config::Config(int ss,
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
BaseSpineCPGControl::Config::Config(ss, tm, om, param, segnum, ct, la, ha,
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
SpineGoalControl::SpineGoalControl(SpineGoalControl::Config config,
                                                std::string args,
                                                std::string resourcePath,
                                                std::string ec,
                                                std::string nc,
                                                std::string fc,
                                                std::string gc) :
BaseSpineCPGControl(config, args, resourcePath, ec, nc),
m_config(config),
feedbackConfigFilename(fc),
// Evolution assumes no pre-processing was done on these names
feedbackEvolution(args + "_fb", fc, resourcePath),
// Will be overwritten by configuration data
feedbackLearning(false),
goalConfigFilename(gc),
// Evolution assumes no pre-processing was done on these names
goalEvolution(args + "_goal", gc, resourcePath),
goalLearning(false)
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
    
    feedbackConfigData.readFile(path + feedbackConfigFilename);
    feedbackLearning = feedbackConfigData.getintvalue("learning");
    
    goalConfigData.readFile(path + goalConfigFilename);
    goalLearning = goalConfigData.getintvalue("learning");
    
}

void SpineGoalControl::onSetup(BaseSpineModelLearning& subject)
{
	m_pCPGSys = new CPGEquationsFB(200);
    //Initialize the Learning Adapters
    nodeAdapter.initialize(&nodeEvolution,
                            nodeLearning,
                            nodeConfigData);
    edgeAdapter.initialize(&edgeEvolution,
                            edgeLearning,
                            edgeConfigData);
    feedbackAdapter.initialize(&feedbackEvolution,
                                feedbackLearning,
                                feedbackConfigData);
    goalAdapter.initialize(&goalEvolution,
                            goalLearning,
                            goalConfigData);
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
    
    const BaseSpineModelGoal* goalSubject = tgCast::cast<BaseSpineModelLearning, BaseSpineModelGoal>(subject);
    std::cout << goalSubject->goalBoxPosition() << std::endl;
}

void SpineGoalControl::onStep(BaseSpineModelLearning& subject, double dt)
{
    m_updateTime += dt;
    if (m_updateTime >= m_config.controlTime)
    {

#if (1)        
        const BaseSpineModelGoal* goalSubject = tgCast::cast<BaseSpineModelLearning, BaseSpineModelGoal>(subject);
        std::vector<double> desComs = getGoalFeedback(goalSubject);
#else // Goal feedback vs others
    #if (1)
            std::vector<double> desComs = getFeedback(subject);

    #else  // cable feedback vs no feedback
            std::size_t numControllers = subject.getNumberofMuslces() * 3;
            
            double descendingCommand = 0.0;
            std::vector<double> desComs (numControllers, descendingCommand);
    #endif
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

void SpineGoalControl::onTeardown(BaseSpineModelLearning& subject)
{
    scores.clear();
    // @todo - check to make sure we ran for the right amount of time
    
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
    feedbackAdapter.endEpisode(scores);
    goalAdapter.endEpisode(scores);
    
    delete m_pCPGSys;
    m_pCPGSys = NULL;
    
    for(size_t i = 0; i < m_allControllers.size(); i++)
    {
        delete m_allControllers[i];
    }
    m_allControllers.clear();    
}

void SpineGoalControl::setupCPGs(BaseSpineModelLearning& subject, array_2D nodeActions, array_4D edgeActions)
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

array_2D SpineGoalControl::scaleNodeActions  
                            (vector< vector <double> > actions)
{
    std::size_t numControllers = nodeConfigData.getintvalue("numberOfControllers");
    std::size_t numActions = nodeConfigData.getintvalue("numberOfActions");
    
    assert( actions.size() == numControllers);
    assert( actions[0].size() == numActions);
    
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

std::vector<double> SpineGoalControl::getFeedback(BaseSpineModelLearning& subject)
{
    // Placeholder
    std:vector<double> feedback;
    // Adapter doesn't use this anyway, so just do zero here for now (will trigger errors if it starts to use it =) )
    const double dt = 0;
    
    const std::vector<tgSpringCableActuator*>& allCables = subject.getAllMuscles();
    
    std::size_t n = allCables.size();
    for(std::size_t i = 0; i != n; i++)
    {
        const tgSpringCableActuator& cable = *(allCables[i]);
        std::vector<double > state = getCableState(cable);
        std::vector< std::vector<double> > actions = feedbackAdapter.step(m_updateTime, state);
        std::vector<double> cableFeedback = transformFeedbackActions(actions, feedbackConfigData);
        
        feedback.insert(feedback.end(), cableFeedback.begin(), cableFeedback.end());
    }
    
    
    return feedback;
}

std::vector<double> SpineGoalControl::getGoalFeedback(const BaseSpineModelGoal* subject)
{

    // Add cable feedback to close the low level loop
    const std::vector<tgSpringCableActuator*>& allCables = subject->getAllMuscles();
    
    std::size_t n = allCables.size();
    std::size_t nA = feedbackConfigData.getintvalue("numberOfActions");
    
    // Placeholder
    std:vector<double> feedback;
    
    // Adapter doesn't use this anyway, so just do zero here for now (will trigger errors if it starts to use it =) )
    const double dt = 0;
    
    // Get heading and generate feedback vector
    std::vector<double> currentPosition = subject->getSegmentCOM(m_config.segmentNumber);
    
    assert(currentPosition.size() == 3);
    
    btVector3 currentPosVector(currentPosition[0], currentPosition[1], currentPosition[2]);
    
    btVector3 goalPosition = subject->goalBoxPosition();
    
    btVector3 desiredHeading = (goalPosition - currentPosVector).normalize();
    
    
#if (1) // Direct to CPG or set impedance controller tensions
    
    int m = subject->getSegments() - 1;
    
    for (int i = 0; i != m; i++)
    {
        // 2D for now to cut down on parameters
        std::vector<double> state;
        state.push_back(desiredHeading.getX());
        state.push_back(desiredHeading.getZ());
        
        assert(state[0] >= -1.0 && state[0] <= 1.0);
        assert(state[1] >= -1.0 && state[1] <= 1.0);
        
        std::vector< std::vector<double> > actions = goalAdapter.step(m_updateTime, state);
        // 24 actions as of 5_12_15, amplitude and phase
        std::vector<double> segmentFeedback = transformFeedbackActions(actions, goalConfigData);
        
        feedback.insert(feedback.end(), segmentFeedback.begin(), segmentFeedback.end());
    }
    
    assert (feedback.size() == n * nA);
    
    
#else
    
    setGoalTensions(subject, desiredHeading);
    
    std::vector<double> zeroFB(n * nA, 0.0);
    
    feedback.insert(feedback.end(), zeroFB.begin(), zeroFB.end());
    
#endif
    assert (feedback.size() == n * nA);

#if (0) //Switch for cable based feedback   
    for(std::size_t i = 0; i != n; i++)
    {
        const tgSpringCableActuator& cable = *(allCables[i]);
        std::vector<double > state = getCableState(cable);
        std::vector< std::vector<double> > actions = feedbackAdapter.step(m_updateTime, state);
        std::vector<double> cableFeedback = transformFeedbackActions(actions, feedbackConfigData);
        
        for (std::size_t j = 0; j != nA; j++)
        {
            feedback[i * nA + j] += cableFeedback[j];
        }
    }
#endif
    
    return feedback;
}

void SpineGoalControl::setGoalTensions(const BaseSpineModelGoal* subject, btVector3& desiredHeading)
{
    std::vector<double> state;
    state.push_back(desiredHeading.getX());
    state.push_back(desiredHeading.getZ());
    
    assert(state[0] >= -1.0 && state[0] <= 1.0);
    assert(state[1] >= -1.0 && state[1] <= 1.0);
    
    std::size_t numActions = goalConfigData.getintvalue("numberOfActions");
    
    std::vector< std::vector<double> > actions = goalAdapter.step(m_updateTime, state);
    
    for (std::size_t i = 0; i < subject->getSegments() - 1; i++)
    {
        for (std::size_t j = 0; j < numActions; j++)
        {
            tgCPGCableControl* cableControl = tgCast::cast<tgCPGActuatorControl, tgCPGCableControl>(m_allControllers[i * numActions + j]);
            cableControl->updateTensionSetpoint(actions[0][j] * m_config.tensFeedback);
        }
    }
}

std::vector<double> SpineGoalControl::getCableState(const tgSpringCableActuator& cable)
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

std::vector<double> SpineGoalControl::transformFeedbackActions(std::vector< std::vector<double> >& actions, configuration& configData)
{
	// Placeholder
	std:vector<double> feedback;
    
    std::size_t numControllers = configData.getintvalue("numberOfControllers");
    std::size_t numActions = configData.getintvalue("numberOfActions");
    
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

double SpineGoalControl::calculateDistanceMoved(const BaseSpineModelGoal* subject) const
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
