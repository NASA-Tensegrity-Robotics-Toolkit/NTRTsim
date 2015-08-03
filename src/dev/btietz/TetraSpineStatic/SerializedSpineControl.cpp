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
 * @file SerializedSpineControl.cpp
 * @brief Contains the implementation of class NestedStructureSineWaves
 * @author Brian Mirletz
 * @version 1.0.0
 * $Id$
 */

// This module
#include "SerializedSpineControl.h"

// Its subject
#include "TetraSpineStaticModel.h"

// NTRTSim
#include "core/tgSpringCableActuator.h"
#include "core/tgBasicActuator.h"
#include "core/tgBaseRigid.h"
#include "controllers/tgImpedanceController.h"
#include "core/abstractMarker.h"
#include "tgcreator/tgUtil.h"

// The C++ Standard Library
#include <stdexcept>
#include <string>

// JSON Serialization
#include "helpers/FileHelpers.h"
#include <json/json.h>

//#define VERBOSE
//#define LOGGING

SerializedSpineControl::Config::Config(std::string fileName)
{

    //BEGIN DESERIALIZING

    Json::Value root; // will contains the root value after parsing.
    Json::Reader reader;
	
	std::string filePath = FileHelpers::getResourcePath("ICRA2015/static/controlVars.json");
		
    bool parsingSuccessful = reader.parse( FileHelpers::getFileString(filePath), root );
    if ( !parsingSuccessful )
    {
        // report to the user the failure and their locations in the document.
        std::cout << "Failed to parse configuration\n"
            << reader.getFormattedErrorMessages();
        throw std::invalid_argument("Bad config filename");
        return;
    }
    // Get the value of the member of root named 'encoding', return 'UTF-8' if there is no
    // such member.
	double kTens = root.get("inside_imp_ten", "UTF-8").asDouble();
	double kPos = root.get("inside_imp_pos", "UTF-8").asDouble();
	double kVel = root.get("inside_imp_vel", "UTF-8").asDouble();
    in_controller = new tgImpedanceController(kTens, kPos, kVel);
    
    kTens = root.get("outside_imp_ten", "UTF-8").asDouble();
	kPos = root.get("outside_imp_pos", "UTF-8").asDouble();
	kVel = root.get("outside_imp_vel", "UTF-8").asDouble();
    out_controller = new tgImpedanceController(kTens, kPos, kVel);

    kTens = root.get("top_imp_ten", "UTF-8").asDouble();
	kPos = root.get("top_imp_pos", "UTF-8").asDouble();
	kVel = root.get("top_imp_vel", "UTF-8").asDouble();
    top_controller = new tgImpedanceController(kTens, kPos, kVel);
	
	rod_edge = root.get("rod_edge", "UTF-8").asDouble();
	rod_front = root.get("rod_front", "UTF-8").asDouble();
    rod_offset = root.get("rod_offset", "UTF-8").asDouble();
	
#if (0)	
    insideLength = sqrt(pow( (rod_edge * sin(M_PI/6)), 2) + pow( (rod_front - rod_offset), 2));
    std::cout << "Computed inside length: " << insideLength << std::endl;
    insideTopLength = insideLength;
    outsideLength = rod_offset;
    outsideTopLength = rod_offset;
#endif
    
    insideTopTens.clear();
    insideTopTens.push_back(root.get("in_top_tens_a", "UTF-8").asDouble());
    insideTopTens.push_back(root.get("in_top_tens_b", "UTF-8").asDouble());
    insideLeftTens.clear();
    insideLeftTens.push_back(root.get("in_left_tens_a", "UTF-8").asDouble());
    insideLeftTens.push_back(root.get("in_left_tens_b", "UTF-8").asDouble());
    insideRightTens.clear();
    insideRightTens.push_back(root.get("in_right_tens_a", "UTF-8").asDouble());
    insideRightTens.push_back(root.get("in_right_tens_b", "UTF-8").asDouble());
    
    outsideTopTens.clear();
    outsideTopTens.push_back(root.get("out_top_tens_a", "UTF-8").asDouble());
    outsideTopTens.push_back(root.get("out_top_tens_b", "UTF-8").asDouble());
    outsideLeftTens.clear();
    outsideLeftTens.push_back(root.get("out_left_tens_a", "UTF-8").asDouble());
    outsideLeftTens.push_back(root.get("out_left_tens_b", "UTF-8").asDouble());
    outsideRightTens.clear();
    outsideRightTens.push_back(root.get("out_right_tens_a", "UTF-8").asDouble());
    outsideRightTens.push_back(root.get("out_right_tens_b", "UTF-8").asDouble());

    insideTopLength.clear();
    insideTopLength.push_back(root.get("in_top_length_a", "UTF-8").asDouble());
    insideTopLength.push_back(root.get("in_top_length_b", "UTF-8").asDouble());
    insideLeftLength.clear();
    insideLeftLength.push_back(root.get("in_left_length_a", "UTF-8").asDouble());
    insideLeftLength.push_back(root.get("in_left_length_b", "UTF-8").asDouble());
    insideRightLength.clear();
    insideRightLength.push_back(root.get("in_right_length_a", "UTF-8").asDouble());
    insideRightLength.push_back(root.get("in_right_length_b", "UTF-8").asDouble());
    
    outsideTopLength.clear();
    outsideTopLength.push_back(root.get("out_top_length_a", "UTF-8").asDouble());
    outsideTopLength.push_back(root.get("out_top_length_b", "UTF-8").asDouble());
    outsideLeftLength.clear();
    outsideLeftLength.push_back(root.get("out_left_length_a", "UTF-8").asDouble());
    outsideLeftLength.push_back(root.get("out_left_length_b", "UTF-8").asDouble());
    outsideRightLength.clear();
    outsideRightLength.push_back(root.get("out_right_length_a", "UTF-8").asDouble());
    outsideRightLength.push_back(root.get("out_right_length_b", "UTF-8").asDouble());
    
	
    
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

SerializedSpineControl::Config::~Config()
{
	delete in_controller;
	delete out_controller;
}


SerializedSpineControl::SerializedSpineControl(std::string fileName) :
m_config(fileName),
segments(1.0),
m_dataObserver("logs/TCData"),
simTime(0.0),
updateTime(0.0),
cycle(0.0),
target(0.0)
{



}

SerializedSpineControl::~SerializedSpineControl()
{
}

void SerializedSpineControl::applyImpedanceControlGeneric(tgImpedanceController* controller,	
										const std::vector<tgSpringCableActuator*> stringList,
										const std::vector<double> stringLengths,
										const std::vector<double> tensions,
										double dt,
										std::size_t phase)
{
	assert(stringList.size() == stringLengths.size() && stringList.size() == tensions.size());
	assert(controller);
	
    for(std::size_t i = 0; i < stringList.size(); i++)
    {
		tgBasicActuator& m_sca = *(tgCast::cast<tgSpringCableActuator, tgBasicActuator>(stringList[i]));
        
        // This will reproduce the same value until simTime is updated. See onStep
		// TODO : consider making inside mod a parameter as well...
        cycle = sin(simTime * m_config.cpgFrequency + 2 * m_config.bodyWaves * M_PI * i / (segments) + m_config.phaseOffsets[phase]);
        target = m_config.offsetSpeed + cycle*m_config.cpgAmplitude;
        
        double setTension = controller->controlTension(m_sca,
                                                        dt,
                                                        stringLengths[i],
                                                        tensions[i],
                                                        target
                                                        );
#ifdef VERBOSE // Conditional compile for verbose control
        std::cout << "Top Outside String " << i << " com tension " << setTension
        << " act tension " << stringList[i]->getMuscle()->getTension()
        << " length " << stringList[i]->getMuscle()->getActualLength() << std::endl;
#endif
    }    
}

void SerializedSpineControl::onSetup(BaseSpineModelLearning& subject)
{
#ifdef LOGGING // Conditional compile for data logging    
    m_dataObserver.onSetup(subject);
#endif  
	

	// Setup initial lengths
	std::vector<tgSpringCableActuator*> stringList;
#if (0)
	stringList = subject.getMuscles("inner top");
	m_config.insideTopLength.clear();
	for(std::size_t i = 0; i < stringList.size(); i++)
    {
		m_config.insideTopLength.push_back(stringList[i]->getStartLength());
	}
	
	stringList = subject.getMuscles("inner left");
	 m_config.insideLeftLength.clear();
	for(std::size_t i = 0; i < stringList.size(); i++)
    {
		m_config.insideLeftLength.push_back(stringList[i]->getStartLength());
	}
	
	stringList = subject.getMuscles("inner right");
	m_config.insideRightLength.clear();
	for(std::size_t i = 0; i < stringList.size(); i++)
    {
		m_config.insideRightLength.push_back(stringList[i]->getStartLength());
	}
#endif
	
#if (0)
	stringList = subject.getMuscles("outer top");
	m_config.outsideTopLength.clear();
	for(std::size_t i = 0; i < stringList.size(); i++)
    {
		m_config.outsideTopLength.push_back(stringList[i]->getStartLength());
	}
	
	stringList = subject.getMuscles("outer left");
	 m_config.outsideLeftLength.clear();
	for(std::size_t i = 0; i < stringList.size(); i++)
    {
		m_config.outsideLeftLength.push_back(stringList[i]->getStartLength());
	}
	
	stringList = subject.getMuscles("outer right");
	m_config.outsideRightLength.clear();
	for(std::size_t i = 0; i < stringList.size(); i++)
    {
		m_config.outsideRightLength.push_back(stringList[i]->getStartLength());
	}
#endif
}

void SerializedSpineControl::onStep(BaseSpineModelLearning& subject, double dt)
{

    updateTime += dt;

	
	if (updateTime >= 1.0/m_config.updateFrequency)
	{
		simTime += updateTime;
		
		
#ifdef LOGGING // Conditional compile for data logging        
        m_dataObserver.onStep(subject, updateTime);
#endif
		updateTime = 0.0;
	}
	
	segments = subject.getSegments();
#if (1)	
	applyImpedanceControlGeneric(m_config.top_controller, 
									subject.getMuscles("inner top"), 
									m_config.insideTopLength, 
									m_config.insideTopTens,
									dt,
									0);
									
	applyImpedanceControlGeneric(m_config.in_controller,
									subject.getMuscles("inner left"),
									m_config.insideLeftLength, 
									m_config.insideLeftTens,
									dt,
									1);
									
	applyImpedanceControlGeneric(m_config.in_controller,
									subject.getMuscles("inner right"),
									m_config.insideRightLength, 
									m_config.insideRightTens,
									dt,
									2);
									
	applyImpedanceControlGeneric(m_config.top_controller, 
									subject.getMuscles("outer top"), 
									m_config.outsideTopLength, 
									m_config.outsideTopTens,
									dt,
									0);
									
	applyImpedanceControlGeneric(m_config.out_controller,
									subject.getMuscles("outer left"),
									m_config.outsideLeftLength, 
									m_config.outsideLeftTens,
									dt,
									1);
									
	applyImpedanceControlGeneric(m_config.out_controller,
									subject.getMuscles("outer right"),
									m_config.outsideRightLength, 
									m_config.outsideRightTens,
									dt,
									2);
	
#endif	
	std::vector<tgBaseRigid*> rigids = subject.getAllRigids();
	btRigidBody* seg1Body = rigids[0]->getPRigidBody();
	btRigidBody* seg2Body = rigids[15]->getPRigidBody();
	btRigidBody* seg3Body = rigids[29]->getPRigidBody();
	
	const abstractMarker marker = subject.getMarkers()[0];
	const abstractMarker marker2 = subject.getMarkers()[1];
	const abstractMarker marker3 = subject.getMarkers()[2];
	const abstractMarker marker4 = subject.getMarkers()[3];
	const abstractMarker marker5 = subject.getMarkers()[4];
	
	btVector3 force(0.0, 0.0, 0.0);
	// 2 kg times gravity
	if (simTime > 30.0 && simTime < 35.0)
	{
		force = btVector3(0.0, 0.0, 2.3 * 981.0 * (simTime - 30) / 5.0);
	}
	else if (simTime >= 35.0 && simTime < 40.0)
	{
		force = btVector3(0.0, 0.0, 2.3 * 981.0);
	}
	else if (simTime >= 40.0 && simTime < 45.0)
	{
		force = btVector3(0.0, 0.0, 2.3 * 981.0 * (45 - simTime)/5.0);
	}
	else
	{
		force = btVector3(0.0, 0.0, 0.0);
	}
	seg1Body->applyForce(force, marker.getRelativePosition());
	seg2Body->applyForce(-force / 2.0 * 0.9, marker2.getRelativePosition());
	seg3Body->applyForce(-force / 2.0 * 1.1, marker5.getRelativePosition());
	//seg2Body->applyForce(-force / 2.0, marker4.getRelativePosition());
}
    

