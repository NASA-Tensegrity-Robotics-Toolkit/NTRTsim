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
 * @file SpineOnlineControl.cpp
 * @brief A controller for the template class BaseSpineModelLearning
 * @author Brian Mirletz
 * @version 1.1.0
 * $Id$
 */

#include "SpineOnlineControl.h"

#include "dev/btietz/TC_goal/BaseSpineModelGoal.h"

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

SpineOnlineControl::Config::Config(int ss,
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
                                        double tf,
                                        double feedTime ) :
SpineGoalControl::Config::Config(ss, tm, om, param, segnum, ct, la, ha,
                                    lp, hp, kt, kp, kv, def, cl, lf, hf,
                                    ffMin, ffMax, afMin, afMax, pfMin, pfMax, tf),
feedbackTime(feedTime)
{
    
}
/**
 * Defining the adapters here assumes the controller is around and
 * attached for the lifecycle of the learning runs. I.E. that the setup
 * and teardown functions are used for tgModel
 */
SpineOnlineControl::SpineOnlineControl(SpineOnlineControl::Config config,
                                                std::string args,
                                                std::string resourcePath,
                                                std::string ec,
                                                std::string nc,
                                                std::string fc,
                                                std::string gc) :
SpineGoalControl(config, args, resourcePath, ec, nc, fc, gc),
m_config(config),
m_feedbackControlTime(0.0)
{
}

void SpineOnlineControl::onSetup(BaseSpineModelLearning& subject)
{
    m_feedbackControlTime = 0.0;
    
    const BaseSpineModelGoal* goalSubject = tgCast::cast<BaseSpineModelLearning, BaseSpineModelGoal>(subject);
    
    m_lastGoalDist = getGoalDist(goalSubject);
    m_controllerStartDist = m_lastGoalDist;
    
    SpineGoalControl::onSetup(subject);
}

void SpineOnlineControl::onStep(BaseSpineModelLearning& subject, double dt)
{
    m_updateTime += dt;
    m_feedbackControlTime += dt;
    if (m_updateTime >= m_config.controlTime)
    {
        if(m_feedbackControlTime > m_config.feedbackTime)
        {
            const BaseSpineModelGoal* goalSubject = tgCast::cast<BaseSpineModelLearning, BaseSpineModelGoal>(subject);
    
            const double dist = getGoalDist(goalSubject);
            
            if (dist > m_lastGoalDist)
            {
#if (0)
                // Moved away from the goal, get a new controller
                std::vector<double> tempScores;
                tempScores.push_back(m_controllerStartDist - dist);
                tempScores.push_back(0.0);
                goalAdapter.endEpisode(tempScores);
                
                goalAdapter.initialize(&goalEvolution,
                        goalLearning,
                        goalConfigData);
                m_controllerStartDist = dist;
#else
                throw std::runtime_error("Moved away from goal");
#endif
            }
            
            m_feedbackControlTime = 0;
            m_lastGoalDist = dist;
        }
#if (1)        
        const BaseSpineModelGoal* goalSubject = tgCast::cast<BaseSpineModelLearning, BaseSpineModelGoal>(subject);
        std::vector<double> desComs = getGoalFeedback(goalSubject);
#else 
        std::vector<double> desComs = getFeedback(subject);

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

void SpineOnlineControl::onTeardown(BaseSpineModelLearning& subject)
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
    
    const double dist = getGoalDist(goalSubject);
    std::vector<double> tempScores;
    tempScores.push_back(m_controllerStartDist - dist);
    tempScores.push_back(0.0);
    goalAdapter.endEpisode(tempScores);
    
    delete m_pCPGSys;
    m_pCPGSys = NULL;
    
    for(size_t i = 0; i < m_allControllers.size(); i++)
    {
        delete m_allControllers[i];
    }
    m_allControllers.clear();    
}

double SpineOnlineControl::getGoalDist(const BaseSpineModelGoal* subject) const
{
    // TODO: consider comparing all segments instead of just the one specified by config
    std::vector<double> finalConditions = subject->getSegmentCOM(m_config.segmentNumber);
  
    const btVector3 goalPos = subject->goalBoxPosition();
    
    double x= finalConditions[0] - goalPos.getX();
    double z= finalConditions[2] - goalPos.getZ();
    double distanceNew=sqrt(x*x + z*z);
    
    return distanceNew;
}
