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
 * @file colSpinesine.cpp
 * @brief Controller for TetraSpineLearningModel
 * @author Brian Mirletz
 * @date November 2014
 * @version 1.0.0
 * $Id$
 */

#include "colSpineSine.h"

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
colSpineSine::colSpineSine(std::string args,
                            std::string resourcePath) :
m_dataObserver("logs/TCData")
{    
    if (resourcePath != "")
    {
        controlFilePath = FileHelpers::getResourcePath(resourcePath);
    }
    else
    {
        controlFilePath = "";
    }
    
    controlFilename = controlFilePath + args;
    
}

void colSpineSine::onSetup(BaseSpineModelLearning& subject)
{

    initConditions = subject.getSegmentCOM(0);
    
    setupWaves(subject);
    
#ifdef LOGGING // Conditional compile for data logging    
    m_dataObserver.onSetup(subject);
#endif    
    
  
    m_updateTime = 0.0;
}

void colSpineSine::onStep(BaseSpineModelLearning& subject, double dt)
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

void colSpineSine::onTeardown(BaseSpineModelLearning& subject)
{
    
    std::vector<double> finalConditions = subject.getSegmentCOM(0);
    
    const double newX = finalConditions[0];
    const double newZ = finalConditions[2];
    const double oldX = initConditions[0];
    const double oldZ = initConditions[2];
    
    const double distanceMoved = sqrt((newX-oldX) * (newX-oldX) + 
                                        (newZ-oldZ) * (newZ-oldZ));
    
    std::cout << "Dist travelled " << distanceMoved << std::endl;
    
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
    
    root["scores"] = distanceMoved;
    
    std::ofstream payloadLog;
    payloadLog.open(controlFilename.c_str(),std::ofstream::out);
    
    payloadLog << root << std::endl;
    
    for(size_t i = 0; i < m_sineControllers.size(); i++)
    {
		delete m_sineControllers[i];
	}
	m_sineControllers.clear();
}

void colSpineSine::setupWaves(BaseSpineModelLearning& subject)
{
	std::vector <tgSpringCableActuator*> allMuscles = subject.getAllMuscles();
    
    double tension;
    double kPosition;
    double kVelocity;
    double controlLength;

	double amplitude;
	double phase;
	double phaseOffset;
	double offset;
		
	Json::Value root; // will contains the root value after parsing.
    Json::Reader reader;

    bool parsingSuccessful = reader.parse( FileHelpers::getFileString(controlFilename.c_str()), root );
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
    double bodywaves = root.get("body_waves", "UTF-8").asDouble() * 2.0 * M_PI / (allMuscles.size() / 6.0);
    
    for (std::size_t i = 0; i < allMuscles.size(); i++)
    {
		if (allMuscles[i]->hasTag("inner top"))
        {
			tension = 100.0;
            kPosition = 600.0;
            
            controlLength = allMuscles[i]->getStartLength();

			amplitude = root.get("in_top_amp_a", "UTF-8").asDouble();
			/// @todo get top, left, right offset, add bodywaves back in.
			phase = i * bodywaves + root.get("in_top_offset", "UTF-8").asDouble();
			kVelocity = 50.0;

		}
        else if (allMuscles[i]->hasTag("outer top"))
        {
            tension = 200.0;
            kPosition = 500.0;
            kVelocity = 100.0;
            

			amplitude = root.get("out_top_amp_a", "UTF-8").asDouble();
			phase = i * bodywaves + root.get("out_top_offset", "UTF-8").asDouble();
			controlLength = allMuscles[i]->getStartLength();

        }
        else if (allMuscles[i]->hasTag("inner left"))
        {
            tension = 100.0;
            kPosition = 600.0;
            kVelocity = 50.0;
            controlLength = allMuscles[i]->getStartLength();

			amplitude = root.get("in_bottom_amp_a", "UTF-8").asDouble();
			phase = i * bodywaves + root.get("in_left_offset", "UTF-8").asDouble();

        }
        else if (allMuscles[i]->hasTag("outer left"))
        {
			tension = 50.0;
            kPosition = 500.0;
            kVelocity = 100.0;
            controlLength = allMuscles[i]->getStartLength();

			amplitude = root.get("out_bottom_amp_a", "UTF-8").asDouble();
			phase = i * bodywaves + root.get("out_left_offset", "UTF-8").asDouble();
			
			//2 * bodyWaves * M_PI * i / (segments) + phaseOffsets[phase]
			
		}
        else if (allMuscles[i]->hasTag("inner right"))
        {
            tension = 100.0;
            kPosition = 600.0;
            kVelocity = 50.0;
            controlLength = allMuscles[i]->getStartLength();

			amplitude = root.get("in_bottom_amp_a", "UTF-8").asDouble();
			phase = i * bodywaves + root.get("in_right_offset", "UTF-8").asDouble();

        }
        else if (allMuscles[i]->hasTag("outer right"))
        {
			tension = 50.0;
            kPosition = 500.0;
            kVelocity = 100.0;
            controlLength = allMuscles[i]->getStartLength();

			amplitude = root.get("out_bottom_amp_a", "UTF-8").asDouble();
			phase = i * bodywaves + root.get("out_right_offset", "UTF-8").asDouble();
			
			//2 * bodyWaves * M_PI * i / (segments) + phaseOffsets[phase]
			
		}
		else
		{
			throw std::runtime_error("Missing tags!");
		}

        tgImpedanceController* p_ipc = new tgImpedanceController( tension,
                                                        kPosition,
                                                        kVelocity);
        
        offset = 0.0;
        
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

