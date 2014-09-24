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
#include "SerializedStaticControl.h"

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

SerializedStaticControl::Config::Config(std::string fileName)
{

    //BEGIN DESERIALIZING

    Json::Value root; // will contains the root value after parsing.
    Json::Reader reader;

    bool parsingSuccessful = reader.parse( FileHelpers::getFileString("controlVarsStatic.json"), root );
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
	
    tensParams.clear();
    tensParams.push_back(root.get("in_top_tens_a", "UTF-8").asDouble());
    tensParams.push_back(root.get("in_top_tens_b", "UTF-8").asDouble());
    tensParams.push_back(root.get("in_left_tens_a", "UTF-8").asDouble());
    tensParams.push_back(root.get("in_left_tens_b", "UTF-8").asDouble());
    tensParams.push_back(root.get("in_right_tens_a", "UTF-8").asDouble());
    tensParams.push_back(root.get("in_right_tens_b", "UTF-8").asDouble());
    tensParams.push_back(root.get("out_top_tens_a", "UTF-8").asDouble());
    tensParams.push_back(root.get("out_top_tens_b", "UTF-8").asDouble());
    tensParams.push_back(root.get("out_left_tens_a", "UTF-8").asDouble());
    tensParams.push_back(root.get("out_left_tens_b", "UTF-8").asDouble());
    tensParams.push_back(root.get("out_right_tens_a", "UTF-8").asDouble());
    tensParams.push_back(root.get("out_right_tens_b", "UTF-8").asDouble());
    
    lengthParams.clear();
    lengthParams.push_back(root.get("in_top_length_a", "UTF-8").asDouble());
    lengthParams.push_back(root.get("in_top_length_b", "UTF-8").asDouble());
    lengthParams.push_back(root.get("in_left_length_a", "UTF-8").asDouble());
    lengthParams.push_back(root.get("in_left_length_b", "UTF-8").asDouble());
    lengthParams.push_back(root.get("in_right_length_a", "UTF-8").asDouble());
    lengthParams.push_back(root.get("in_right_length_b", "UTF-8").asDouble());
    lengthParams.push_back(root.get("out_top_length_a", "UTF-8").asDouble());
    lengthParams.push_back(root.get("out_top_length_b", "UTF-8").asDouble());
    lengthParams.push_back(root.get("out_left_length_a", "UTF-8").asDouble());
    lengthParams.push_back(root.get("out_left_length_b", "UTF-8").asDouble());
    lengthParams.push_back(root.get("out_right_length_a", "UTF-8").asDouble());
    lengthParams.push_back(root.get("out_right_length_b", "UTF-8").asDouble());
	
	
		rod_edge = root.get("rod_edge", "UTF-8").asDouble();
	rod_front = root.get("rod_front", "UTF-8").asDouble();
    rod_offset = root.get("rod_offset", "UTF-8").asDouble();
	
	updateFrequency = root.get("updateFrequency", "UTF-8").asDouble();
	//END SERIALIZING
	
	/// @todo verify data!
}

SerializedStaticControl::Config::~Config()
{

}


SerializedStaticControl::SerializedStaticControl(std::string fileName) :
m_config(fileName),
segments(1.0),
m_dataObserver("logs/TCData"),
simTime(0.0),
updateTime(0.0),
cycle(0.0),
target(0.0)
{



}

SerializedStaticControl::~SerializedStaticControl()
{
}


void SerializedStaticControl::onSetup(BaseSpineModelLearning& subject)
{
#ifdef LOGGING // Conditional compile for data logging    
    m_dataObserver.onSetup(subject);
#endif  

#if (1)
	// Setup initial lengths
	std::vector<tgLinearString*> stringList;
	int j = 0;
	
	stringList = subject.getMuscles("inner top");
	for(std::size_t i = 0; i < stringList.size(); i++)
    {
		const double stiffness = stringList[i]->getMuscle()->getCoefK();
		stringList[i]->setRestLengthSingleStep(m_config.lengthParams[j] - m_config.tensParams[j]/stiffness);
		j++;
	}
	
	stringList = subject.getMuscles("inner left");
	for(std::size_t i = 0; i < stringList.size(); i++)
    {
		const double stiffness = stringList[i]->getMuscle()->getCoefK();
		stringList[i]->setRestLengthSingleStep(m_config.lengthParams[j] - m_config.tensParams[j]/stiffness);
		j++;
	}
	stringList = subject.getMuscles("inner right");
	for(std::size_t i = 0; i < stringList.size(); i++)
    {
		const double stiffness = stringList[i]->getMuscle()->getCoefK();
		stringList[i]->setRestLengthSingleStep(m_config.lengthParams[j] - m_config.tensParams[j]/stiffness);
		j++;
	}
	
	stringList = subject.getMuscles("outer top");
	for(std::size_t i = 0; i < stringList.size(); i++)
    {
		const double stiffness = stringList[i]->getMuscle()->getCoefK();
		stringList[i]->setRestLengthSingleStep(m_config.lengthParams[j] - m_config.tensParams[j]/stiffness);
		j++;
	}
	
	stringList = subject.getMuscles("outer left");
	for(std::size_t i = 0; i < stringList.size(); i++)
    {
		const double stiffness = stringList[i]->getMuscle()->getCoefK();
		stringList[i]->setRestLengthSingleStep(m_config.lengthParams[j] - m_config.tensParams[j]/stiffness);
		j++;
	}
	
	stringList = subject.getMuscles("outer right");
	for(std::size_t i = 0; i < stringList.size(); i++)
    {
		const double stiffness = stringList[i]->getMuscle()->getCoefK();
		stringList[i]->setRestLengthSingleStep(m_config.lengthParams[j] - m_config.tensParams[j]/stiffness);
		j++;
	}
#endif	
}

void SerializedStaticControl::onStep(BaseSpineModelLearning& subject, double dt)
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
#if (0)	
	applyImpedanceControlTopInside(subject.getMuscles("inner top"), dt, 0);
	applyImpedanceControlInside(subject.getMuscles("inner left") , dt, 1);
	applyImpedanceControlInside(subject.getMuscles("inner right"), dt, 2);
	
	applyImpedanceControlTopOutside(subject.getMuscles("outer top"), dt, 0);
	applyImpedanceControlOutside(subject.getMuscles("outer left"), dt, 1);
	applyImpedanceControlOutside(subject.getMuscles("outer right"), dt, 2);
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
	seg2Body->applyForce(-force / 2.0, marker2.getRelativePosition());
	seg3Body->applyForce(-force / 2.0, marker5.getRelativePosition());
	//seg2Body->applyForce(-force / 2.0, marker4.getRelativePosition());
}
    

