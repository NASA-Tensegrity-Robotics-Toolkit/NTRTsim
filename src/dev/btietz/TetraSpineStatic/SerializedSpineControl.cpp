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
#include "core/tgLinearString.h"
#include "core/tgBaseRigid.h"
#include "core/ImpedanceControl.h"
#include "core/abstractMarker.h"
#include "core/Muscle2P.h"
#include "tgcreator/tgUtil.h"

// The C++ Standard Library
#include <stdexcept>
#include <string>

// JSON Serialization
#include "helpers/FileHelpers.h"
#include <json/json.h>

//#define VERBOSE
#define LOGGING

SerializedSpineControl::Config::Config(std::string fileName)
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
    in_controller = new ImpedanceControl(kTen, kPos, kVel);
    
    kTen = root.get("outside_imp_ten", "UTF-8").asDouble();
	kPos = root.get("outside_imp_pos", "UTF-8").asDouble();
	kVel = root.get("outside_imp_vel", "UTF-8").asDouble();
    out_controller = new ImpedanceControl(kTen, kPos, kVel);

    kTen = root.get("top_imp_ten", "UTF-8").asDouble();
	kPos = root.get("top_imp_pos", "UTF-8").asDouble();
	kVel = root.get("top_imp_vel", "UTF-8").asDouble();
    top_controller = new ImpedanceControl(kTen, kPos, kVel);
	
	rod_edge = root.get("rod_edge", "UTF-8").asDouble();
	rod_front = tgUtil::round(std::sqrt(3.0)/2 * rod_edge);
    rod_offset = root.get("rod_offset", "UTF-8").asDouble();
	
#if (0)	
    insideLength = sqrt(pow( (rod_edge / cos(M_PI/6)), 2) + pow( (rod_front - rod_offset), 2));
    outsideLength = rod_offset;
#else
	insideLength = root.get("inside_length", "UTF-8").asDouble();	
	outsideLength = root.get("outside_length", "UTF-8").asDouble();	
	
#endif
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

void SerializedSpineControl::applyImpedanceControlInside(const std::vector<tgLinearString*> stringList,
                                                            double dt,
                                                            std::size_t phase)
{
    for(std::size_t i = 0; i < stringList.size(); i++)
    {
		// This will reproduce the same value until simTime is updated. See onStep
        cycle = sin(simTime * m_config.cpgFrequency + 2 * m_config.bodyWaves * M_PI * i / (segments) + m_config.phaseOffsets[phase]);
        target = m_config.offsetSpeed + cycle*m_config.cpgAmplitude;
        
		
        double setTension = m_config.in_controller->control(stringList[i],
																dt,
																m_config.insideLength,
																m_config.insideMod * target
																);
#ifdef VERBOSE // Conditional compile for verbose control
        std::cout << "Inside String " << i
        << " phase " << phase
         << " tension " << setTension
        << " act tension " << stringList[i]->getMuscle()->getTension()
        << " length " << stringList[i]->getMuscle()->getActualLength() << std::endl;
#endif
    }    
}

void SerializedSpineControl::applyImpedanceControlTopInside(const std::vector<tgLinearString*> stringList,
                                    double dt,
                                    std::size_t phase)
{
    for(std::size_t i = 0; i < stringList.size(); i++)
    {
		// This will reproduce the same value until simTime is updated. See onStep
        cycle = sin(simTime * m_config.cpgFrequency + 2 * m_config.bodyWaves * M_PI * i / (segments) + m_config.phaseOffsets[phase]);
        target = m_config.offsetSpeed + cycle*m_config.cpgAmplitude;
        
        double setTension = m_config.top_controller->control(stringList[i],
																dt,
																m_config.insideLength,
																m_config.insideMod * target
																);
#ifdef VERBOSE																
        std::cout << "Top Inside String " << i << " com tension " << setTension
        << " act tension " << stringList[i]->getMuscle()->getTension()
        << " length " << stringList[i]->getMuscle()->getActualLength() << std::endl;
#endif
    }    
}

void SerializedSpineControl::applyImpedanceControlOutside(const std::vector<tgLinearString*> stringList,
                                                            double dt,
                                                            std::size_t phase)
{
    for(std::size_t i = 0; i < stringList.size(); i++)
    {
		// This will reproduce the same value until simTime is updated. See onStep
        cycle = sin(simTime * m_config.cpgFrequency + 2 * m_config.bodyWaves * M_PI * i / (segments) + m_config.phaseOffsets[phase]);
        target = m_config.offsetSpeed + cycle*m_config.cpgAmplitude;
        
        double setTension = m_config.out_controller->control(stringList[i],
																dt,
																m_config.outsideLength,
																target
																);
#ifdef VERBOSE // Conditional compile for verbose control
        std::cout << "Outside String " << i
        << " phase " << phase
         << " com tension " << setTension
        << " act tension " << stringList[i]->getMuscle()->getTension()
        << " length " << stringList[i]->getMuscle()->getActualLength() << std::endl;
#endif
    }    
}

void SerializedSpineControl::applyImpedanceControlTopOutside(const std::vector<tgLinearString*> stringList,
                                    double dt,
                                    std::size_t phase)
{
    for(std::size_t i = 0; i < stringList.size(); i++)
    {
		// This will reproduce the same value until simTime is updated. See onStep
        cycle = sin(simTime * m_config.cpgFrequency + 2 * m_config.bodyWaves * M_PI * i / (segments) + m_config.phaseOffsets[phase]);
        target = m_config.offsetSpeed + cycle*m_config.cpgAmplitude;
        
        double setTension = m_config.top_controller->control(stringList[i],
																dt,
																m_config.outsideLength,
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
	
	applyImpedanceControlTopInside(subject.getMuscles("inner top"), dt, 0);
	applyImpedanceControlInside(subject.getMuscles("inner left") , dt, 1);
	applyImpedanceControlInside(subject.getMuscles("inner right"), dt, 2);
	
	applyImpedanceControlTopOutside(subject.getMuscles("outer top"), dt, 0);
	applyImpedanceControlOutside(subject.getMuscles("outer left"), dt, 1);
	applyImpedanceControlOutside(subject.getMuscles("outer right"), dt, 2);
	
	std::vector<tgBaseRigid*> rigids = subject.getAllRigids();
	btRigidBody* seg1Body = rigids[0]->getPRigidBody();
	btRigidBody* seg2Body = rigids[6]->getPRigidBody();
	
	const abstractMarker marker = subject.getMarkers()[0];
	const abstractMarker marker2 = subject.getMarkers()[1];
	const abstractMarker marker3 = subject.getMarkers()[2];
	const abstractMarker marker4 = subject.getMarkers()[3];
	
	btVector3 force(0.0, 0.0, 0.0);
	// 2 kg times gravity
	if (simTime > 30.0 && simTime < 32.0)
	{
		force = btVector3(0.0, 0.0, 2*981.0 * ((simTime - 30) / 2.0));
	}
	else if (simTime >= 32.0 && simTime < 38.0)
	{
		force = btVector3(0.0, 0.0, 2*981.0);
	}
	else if (simTime >= 38.0 && simTime < 40.0)
	{
		force = btVector3(0.0, 0.0, 2*981.0 * (40 - simTime)/ 2.0);
	}
	else
	{
		force = btVector3(0.0, 0.0, 0.0);
	}
	seg1Body->applyForce(force, marker.getRelativePosition());
	seg2Body->applyForce(-force, marker2.getRelativePosition());
	//seg2Body->applyForce(-force / 2.0, marker3.getRelativePosition());
	//seg2Body->applyForce(-force / 2.0, marker4.getRelativePosition());
}
    

