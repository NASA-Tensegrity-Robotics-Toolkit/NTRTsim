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
 * @file htSpineJSON.cpp
 * @brief Controller for TetraSpineLearningModel
 * @author Brian Tietz
 * @date May 2014
 * @version 1.0.0
 * $Id$
 */

#include "htSpineSine.h"

#include <string>


// Should include tgString, but compiler complains since its been
// included from TetraSpineLearningModel. Perhaps we should move things
// to a cpp over there
#include "core/tgSpringCableActuator.h"
#include "controllers/tgImpedanceController.h"

#include "learning/AnnealEvolution/AnnealEvolution.h"
#include "learning/Configuration/configuration.h"

#include "dev/btietz/hardwareSineWaves/tgSineStringControl.h"

// JSON Serialization
#include "helpers/FileHelpers.h"
#include <json/json.h>

// The C++ Standard Library
#include <stdexcept>

//#define LOGGING

/**
 * Defining the adapters here assumes the controller is around and
 * attached for the lifecycle of the learning runs. I.E. that the setup
 * and teardown functions are used for tgModel
 */
htSpineSine::htSpineSine() :
m_dataObserver("logs/TCData")
{    
}

void htSpineSine::onSetup(BaseSpineModelLearning& subject)
{

    setupWaves(subject);
    
#ifdef LOGGING // Conditional compile for data logging    
    m_dataObserver.onSetup(subject);
#endif    
    
  
    m_updateTime = 0.0;
}

void htSpineSine::onStep(BaseSpineModelLearning& subject, double dt)
{
	/// Basically nothing to do. Sine controllers will take care of themselves
    m_updateTime += dt;
    if (m_updateTime >= m_controlTime)
    {

#ifdef LOGGING // Conditional compile for data logging        
        m_dataObserver.onStep(subject, m_updateTime);
#endif
        m_updateTime = 0;
    }
}

void htSpineSine::onTeardown(BaseSpineModelLearning& subject)
{

    for(size_t i = 0; i < m_sineControllers.size(); i++)
    {
		delete m_sineControllers[i];
	}
	m_sineControllers.clear();
}

void htSpineSine::setupWaves(BaseSpineModelLearning& subject)
{
	std::vector <tgSpringCableActuator*> allMuscles = subject.getAllMuscles();
    
    double tension;
    double kPosition;
    double kVelocity;
    double controlLength;

	double amplitude;
	double phase;
	double offset;
	
	Json::Value root; // will contains the root value after parsing.
    Json::Reader reader;

    bool parsingSuccessful = reader.parse( FileHelpers::getFileString("controlVars.json"), root );
    if ( !parsingSuccessful )
    {
        // report to the user the failure and their locations in the document.
        std::cout << "Failed to parse configuration\n"
            << reader.getFormattedErrorMessages();
        /// @todo should this throw an exception instead??
        throw std::invalid_argument("Bad JSON filename");
    }
    
    m_controlTime = root.get("updateFrequency", "UTF-8").asDouble();
    double frequency = root.get("cpg_frequency", "UTF-8").asDouble();
    
    for (std::size_t i = 0; i < allMuscles.size(); i++)
    {
		if (allMuscles[i]->hasTag("inner top"))
        {
			tension = 2400.0;
            kPosition = 500.0;
            
            //controlLength = allMuscles[i]->getStartLength();
            
            if (allMuscles[i]->hasTag("seg1"))
            {
				amplitude = root.get("in_top_amp_a", "UTF-8").asDouble();
				phase = root.get("front_offset", "UTF-8").asDouble();
				controlLength = 19.0;
                kVelocity = 150;
			}
			else if(allMuscles[i]->hasTag("seg2"))
			{
				amplitude = root.get("in_top_amp_b", "UTF-8").asDouble();
				phase = root.get("back_offset", "UTF-8").asDouble();
				controlLength = 18.0;
                kVelocity = 150.0;
			}
			else
			{
				throw std::runtime_error("Missing tags!");
			}
		}
        else if (allMuscles[i]->hasTag("outer top"))
        {
            tension = 1000.0;
            kPosition = 500.0;
            kVelocity = 150.0;
            
            if (allMuscles[i]->hasTag("seg1"))
            {
				amplitude = root.get("out_top_amp_a", "UTF-8").asDouble();
				phase = root.get("front_offset", "UTF-8").asDouble();
			
				controlLength = 18.5;
			}
			else if(allMuscles[i]->hasTag("seg2"))
			{
				amplitude = root.get("out_top_amp_b", "UTF-8").asDouble();
				phase = root.get("back_offset", "UTF-8").asDouble();
				controlLength = 19.8;
			}
			else
			{
				throw std::runtime_error("Missing tags!");
			}
        }
        else if (allMuscles[i]->hasTag("inner"))
        {
            tension = 1000.0;
            kPosition = 300.0;
            kVelocity = 100.0;
            //controlLength = allMuscles[i]->getStartLength();
            
            if (allMuscles[i]->hasTag("seg1"))
            {
				amplitude = root.get("in_bottom_amp_a", "UTF-8").asDouble();
				phase = root.get("front_offset", "UTF-8").asDouble();
                controlLength = 21.5;
			}
			else if(allMuscles[i]->hasTag("seg2"))
			{
				amplitude = root.get("in_bottom_amp_b", "UTF-8").asDouble();
				phase = root.get("back_offset", "UTF-8").asDouble();
                controlLength = 22.5;
			}
			else
			{
				throw std::runtime_error("Missing tags!");
			}
        }
        else if (allMuscles[i]->hasTag("outer"))
        {
			tension = 500.0;
            kPosition = 100.0;
            kVelocity = 100.0;
            controlLength = 19.0;
            if (allMuscles[i]->hasTag("seg1"))
            {
				amplitude = root.get("out_bottom_amp_a", "UTF-8").asDouble();
				phase = root.get("front_offset", "UTF-8").asDouble();
			}
			else if(allMuscles[i]->hasTag("seg2"))
			{
				amplitude = root.get("out_bottom_amp_b", "UTF-8").asDouble();
				phase = root.get("back_offset", "UTF-8").asDouble();
			}
			else
			{
				throw std::runtime_error("Missing tags!");
			}
		}
		else
		{
			throw std::runtime_error("Missing tags!");
		}

        tgImpedanceController* p_ipc = new tgImpedanceController( tension,
                                                        kPosition,
                                                        kVelocity);
        
        // In Won's convention

        offset = -amplitude;
        
        tgSineStringControl* pStringControl = new tgSineStringControl(m_controlTime,
																		p_ipc,
																		amplitude,
																		frequency,
																		phase,
																		offset,
																		controlLength);

		
		allMuscles[i]->attach(pStringControl);
        m_sineControllers.push_back(pStringControl);
    }
    
    assert(m_sineControllers.size() == allMuscles.size());
  
}

