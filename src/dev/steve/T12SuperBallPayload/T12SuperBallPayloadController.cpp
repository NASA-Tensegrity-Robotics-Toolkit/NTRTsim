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
 * @file T6PrefLengthController.cpp
 * @brief Preferred Length Controller for T6. Constant speed motors are used in muscles to reach preffered length
 * @author Atil Iscen
 * @version 1.0.0
 * $Id$
 */

// This module
#include "T12SuperBallPayloadController.h"
// This application
#include "T12SuperBallPayload.h"
// This library
#include "core/tgBasicActuator.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>

using namespace std;

//Constructor using the model subject and a single pref length for all muscles.
SuperBallPrefLengthController::SuperBallPrefLengthController(const double initialLength)
{
	this->m_initialLengths=initialLength;
	this->m_totalTime=0.0;
}

//Fetch all the muscles and set their preferred length
void SuperBallPrefLengthController::onSetup(T12SuperBallPayload& subject)
{
	const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
	for (size_t i = 0; i < muscles.size(); ++i)
	{
		tgBasicActuator * const pMuscle = muscles[i];
		assert(pMuscle != NULL);
		pMuscle->setControlInput(this->m_initialLengths,0.0001);
	}
}

void SuperBallPrefLengthController::onStep(T12SuperBallPayload& subject, double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    m_totalTime+=dt;

    //Move motors for all the muscles
	const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
	for (size_t i = 0; i < muscles.size(); ++i)
	{
		tgBasicActuator * const pMuscle = muscles[i];
		assert(pMuscle != NULL);
		pMuscle->moveMotors(dt);
	}

	//vector<double> state=getState();
	vector< vector<double> > actions;

	//get the actions (between 0 and 1) from evolution (todo)
	//actions=evolutionAdapter.step(dt,state);

	//instead, generate it here for now!
	for(int i=0;i<24;i++)
	{
		vector<double> tmp;
		for(int j=0;j<2;j++)
		{
			tmp.push_back(0.5);
		}
		actions.push_back(tmp);
	}

	//transform them to the size of the structure
	actions = transformActions(actions);

	//apply these actions to the appropriate muscles according to the sensor values
//	applyActions(subject,actions);

}

//Scale actions according to Min and Max length of muscles.
vector< vector <double> > SuperBallPrefLengthController::transformActions(vector< vector <double> > actions)
{
	double min=6;
	double max=11;
	double range=max-min;
	double scaledAct;
	for(int i=0;i<actions.size();i++)
	{
		for(int j=0;j<actions[i].size();j++)
		{
			scaledAct=actions[i][j]*(range)+min;
			actions[i][j]=scaledAct;
		}
	}
	return actions;
}

//Pick particular muscles (according to the structure's state) and apply the given actions one by one
void SuperBallPrefLengthController::applyActions(T12SuperBallPayload& subject, vector< vector <double> > act)
{
	//Get All the muscles of the subject
	const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
	//Check if the number of the actions match the number of the muscles
	if(act.size() != muscles.size())
	{
		cout<<"Warning: # of muscles: "<< muscles.size() << " != # of actions: "<< act.size()<<endl;
		return;
	}
	//Apply actions (currently in a random order)
	for (size_t i = 0; i < muscles.size(); ++i)
	{
		tgBasicActuator * const pMuscle = muscles[i];
		assert(pMuscle != NULL);
		//cout<<"i: "<<i<<" length: "<<act[i][0]<<endl;
		pMuscle->setControlInput(act[i][0]);
	}
}
