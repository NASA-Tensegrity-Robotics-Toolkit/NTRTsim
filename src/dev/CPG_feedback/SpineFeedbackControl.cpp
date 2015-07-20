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
 * @file SpineFeedbackControl.cpp
 * @brief A controller for the template class BaseSpineModelLearning
 * @author Brian Mirletz
 * @version 1.1.0
 * $Id$
 */

#include "SpineFeedbackControl.h"

#include <string>


// Should include tgString, but compiler complains since its been
// included from BaseSpineModelLearning. Perhaps we should move things
// to a cpp over there
#include "core/tgSpringCableActuator.h"
#include "core/tgBasicActuator.h"
#include "controllers/tgImpedanceController.h"
#include "examples/learningSpines/tgCPGActuatorControl.h"
#include "tgCPGCableControl.h"

#include "helpers/FileHelpers.h"

#include "learning/AnnealEvolution/AnnealEvolution.h"
#include "learning/Configuration/configuration.h"

#include "CPGEquationsFB.h"
#include "CPGNodeFB.h"

//#define LOGGING
#define USE_KINEMATIC

using namespace std;

SpineFeedbackControl::Config::Config(int ss,
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
                                        double pfMax) :
BaseSpineCPGControl::Config::Config(ss, tm, om, param, segnum, ct, la, ha,
                                    lp, hp, kt, kp, kv, def, cl, lf, hf),
freqFeedbackMin(ffMin),
freqFeedbackMax(ffMax),
ampFeedbackMin(afMin),
ampFeedbackMax(afMax),
phaseFeedbackMin(pfMin),
phaseFeedbackMax(pfMax)
{
    
}
/**
 * Defining the adapters here assumes the controller is around and
 * attached for the lifecycle of the learning runs. I.E. that the setup
 * and teardown functions are used for tgModel
 */
SpineFeedbackControl::SpineFeedbackControl(SpineFeedbackControl::Config config,	
                                                std::string args,
                                                std::string resourcePath,
                                                std::string ec,
                                                std::string nc,
                                                std::string fc) :
BaseSpineCPGControl(config, args, resourcePath, ec, nc),
m_config(config),
feedbackConfigFilename(fc),
// Evolution assumes no pre-processing was done on these names
feedbackEvolution(args + "_fb", fc, resourcePath),
// Will be overwritten by configuration data
feedbackLearning(false)
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
    
}

void SpineFeedbackControl::onSetup(BaseSpineModelLearning& subject)
{
	m_pCPGSys = new CPGEquationsFB(100);
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

void SpineFeedbackControl::onStep(BaseSpineModelLearning& subject, double dt)
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
    
    /// @todo add to config
    if (currentHeight > 25 || currentHeight < 1.0)
    {
		/// @todo if bogus, stop trial (reset simulation)
		bogus = true;
		throw std::runtime_error("Height out of range");
	}
}

void SpineFeedbackControl::onTeardown(BaseSpineModelLearning& subject)
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
    feedbackAdapter.endEpisode(scores);
    
    delete m_pCPGSys;
    m_pCPGSys = NULL;
    
    for(size_t i = 0; i < m_allControllers.size(); i++)
    {
        delete m_allControllers[i];
    }
    m_allControllers.clear();    
}

void SpineFeedbackControl::setupCPGs(BaseSpineModelLearning& subject, array_2D nodeActions, array_4D edgeActions)
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

array_2D SpineFeedbackControl::scaleNodeActions  
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

std::vector<double> SpineFeedbackControl::getFeedback(BaseSpineModelLearning& subject)
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
        std::vector<double> cableFeedback = transformFeedbackActions(actions);
        
        feedback.insert(feedback.end(), cableFeedback.begin(), cableFeedback.end());
       
#if (0)
        for (std::size_t j = 0; j < cableFeedback.size(); j++)
        {
            std::cout << cableFeedback[j] << " ";
        }
    }
    std::cout << std::endl;
#else
    }
#endif
    
    return feedback;
}

std::vector<double> SpineFeedbackControl::getCableState(const tgSpringCableActuator& cable)
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

std::vector<double> SpineFeedbackControl::transformFeedbackActions(std::vector< std::vector<double> >& actions)
{
	// Placeholder
	std:vector<double> feedback;
    
    std::size_t numControllers = feedbackConfigData.getintvalue("numberOfControllers");
    std::size_t numActions = feedbackConfigData.getintvalue("numberOfActions");
    
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
