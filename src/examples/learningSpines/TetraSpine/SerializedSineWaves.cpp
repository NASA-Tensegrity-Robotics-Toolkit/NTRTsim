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
 * @file NestedStructureSineWaves.cpp
 * @brief Contains the implementation of class NestedStructureSineWaves
 * @author Brian Tietz
 * @version 1.0.0
 * $Id$
 */

// This module
#include "SerializedSineWaves.h"

// Its subject
#include "TetraSpineLearningModel.h"

// NTRTSim
#include "core/tgSpringCableActuator.h"
#include "controllers/tgImpedanceController.h"
#include "tgcreator/tgUtil.h"
#include "core/tgCast.h"
#include "core/tgBasicActuator.h"

// The C++ Standard Library
#include <stdexcept>
#include <string>

// JSON Serialization
#include "helpers/FileHelpers.h"
#include <json/json.h>

SerializedSineWaves::Config::Config(std::string fileName)
{

    //BEGIN DESERIALIZING

    Json::Value root; // will contains the root value after parsing.
    Json::Reader reader;

    bool parsingSuccessful = reader.parse( FileHelpers::getFileString("controlVars.json"), root );
    if ( !parsingSuccessful )
    {
        // report to the user the failure and their locations in the document.
        std::cout << "Failed to parse configuration\n"
            << reader.getFormattedErrorMessages();
        /// @todo should this throw an exception instead??
        return;
    }
    // Get the value of the member of root named 'encoding', return 'UTF-8' if there is no
    // such member.
	double kTen = root.get("inside_imp_ten", "UTF-8").asDouble();
	double kPos = root.get("inside_imp_pos", "UTF-8").asDouble();
	double kVel = root.get("inside_imp_vel", "UTF-8").asDouble();
    in_controller = new tgImpedanceController(kTen, kPos, kVel);
    
    kTen = root.get("outside_imp_ten", "UTF-8").asDouble();
	kPos = root.get("outside_imp_pos", "UTF-8").asDouble();
	kVel = root.get("outside_imp_vel", "UTF-8").asDouble();
    out_controller = new tgImpedanceController(kTen, kPos, kVel);

    insideLength = root.get("inside_length", "UTF-8").asDouble();
    outsideLength = root.get("outside_length", "UTF-8").asDouble();
    offsetSpeed = root.get("offset_speed", "UTF-8").asDouble();
    cpgAmplitude = root.get("cpg_amplitude", "UTF-8").asDouble();
    cpgFrequency = root.get("cpg_frequency", "UTF-8").asDouble();
    bodyWaves = root.get("bodyWaves", "UTF-8").asDouble();
	insideMod = root.get("insideMod", "UTF-8").asDouble();
	
	updateFrequency = root.get("updateFrequency", "UTF-8").asDouble();
	
    phaseOffsets.clear();
    phaseOffsets.push_back(root.get("top_offset", "UTF-8").asDouble());
    phaseOffsets.push_back(root.get("left_offset", "UTF-8").asDouble());
    phaseOffsets.push_back(root.get("right_offset", "UTF-8").asDouble());
	
	//END SERIALIZING
	
	/// @todo verify data!
}

SerializedSineWaves::Config::~Config()
{
	delete in_controller;
	delete out_controller;
}


SerializedSineWaves::SerializedSineWaves(std::string fileName) :
m_config(fileName),
segments(1.0),
simTime(0.0),
updateTime(0.0),
cycle(0.0),
target(0.0)
{

}

SerializedSineWaves::~SerializedSineWaves()
{
}

void SerializedSineWaves::applyImpedanceControlInside(const std::vector<tgSpringCableActuator*> stringList,
                                                            double dt,
                                                            std::size_t phase)
{
    std::vector<tgBasicActuator* > stringList_ba = tgCast::filter<tgSpringCableActuator, tgBasicActuator>(stringList);
    
    for(std::size_t i = 0; i < stringList_ba.size(); i++)
    {
		// This will reproduce the same value until simTime is updated. See onStep
        cycle = sin(simTime * m_config.cpgFrequency + 2 * m_config.bodyWaves * M_PI * i / (segments) + m_config.phaseOffsets[phase]);
        target = m_config.offsetSpeed + cycle*m_config.cpgAmplitude;
        
		
        double setTension = m_config.in_controller->control(*(stringList_ba[i]),
																dt,
																m_config.insideLength,
																m_config.insideMod * target
																);
        #if (0) // Conditional compile for verbose control
        std::cout << "Inside String " << i << " tension " << setTension
        << " act tension " << stringList[i]->getMuscle()->getTension()
        << " length " << stringList[i]->getMuscle()->getActualLength() << std::endl;
        #endif
    }    
}

void SerializedSineWaves::applyImpedanceControlOutside(const std::vector<tgSpringCableActuator*> stringList,
                                                            double dt,
                                                            std::size_t phase)
{
    std::vector<tgBasicActuator* > stringList_ba = tgCast::filter<tgSpringCableActuator, tgBasicActuator>(stringList);
    
    for(std::size_t i = 0; i < stringList_ba.size(); i++)
    {
		// This will reproduce the same value until simTime is updated. See onStep
        cycle = sin(simTime * m_config.cpgFrequency + 2 * m_config.bodyWaves * M_PI * i / (segments) + m_config.phaseOffsets[phase]);
        target = m_config.offsetSpeed + cycle*m_config.cpgAmplitude;
        
        double setTension = m_config.out_controller->control(*( stringList_ba[i]),
																dt,
																m_config.outsideLength,
																target
																);
        #if(0) // Conditional compile for verbose control
        std::cout << "Outside String " << i << " com tension " << setTension
        << " act tension " << stringList[i]->getMuscle()->getTension()
        << " length " << stringList[i]->getMuscle()->getActualLength() << std::endl;
        #endif
    }    
}

void SerializedSineWaves::onStep(BaseSpineModelLearning& subject, double dt)
{

    updateTime += dt;

	if (updateTime >= 1.0/m_config.updateFrequency)
	{
		simTime += updateTime;
		updateTime = 0.0;
	}
	
	segments = subject.getSegments();
	
	applyImpedanceControlInside(subject.getMuscles("inner top"), dt, 0);
	applyImpedanceControlInside(subject.getMuscles("inner left") , dt, 1);
	applyImpedanceControlInside(subject.getMuscles("inner right"), dt, 2);
	
	applyImpedanceControlOutside(subject.getMuscles("outer top"), dt, 0);
	applyImpedanceControlOutside(subject.getMuscles("outer left"), dt, 1);
	applyImpedanceControlOutside(subject.getMuscles("outer right"), dt, 2);

}
    

