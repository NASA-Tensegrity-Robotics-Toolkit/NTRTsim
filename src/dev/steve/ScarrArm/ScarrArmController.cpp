/*
 * Copyright © 2015, United States Government, as represented by the
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
 * @file ScarrArmController.cpp
 * @brief Preferred Length Controller for ScarrArmModel
 * @author Steven Lessard
 * @version 1.0.0
 * $Id$
 */

// This module
#include "ScarrArmController.h"
// This application
#include "ScarrArmModel.h"
// This library
#include "core/tgBasicActuator.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>

using namespace std;

//Constructor using the model subject and a single pref length for all muscles.
ScarrArmController::ScarrArmController(const double initialLength)
{
	this->m_initialLengths=initialLength;
	this->m_totalTime=0.0;
}

//Fetch all the muscles and set their preferred length
void ScarrArmController::onSetup(ScarrArmModel& subject)
{
    const double dt = 0.0001;
    const double scale = 0.5;
    const double bone_scale = 0.5;
    const double a = 22; //TODO: Currently ulna distal width, needs to be olecranon diameter (not quite, but close to that in length)

    const double olecranonfascia_length = a/std::sqrt(2.0) * scale;
    const double anconeus_length        = a/std::sqrt(2.0) * scale; //TODO: Change
    const double brachioradialis_length = 262 * scale * bone_scale; //TODO: Justify
    const double supportstring_length   = 1 * scale;

	const std::vector<tgBasicActuator*> olecranonfascia = subject.find<tgBasicActuator>("olecranon");
	const std::vector<tgBasicActuator*> anconeus        = subject.find<tgBasicActuator>("anconeus");
	const std::vector<tgBasicActuator*> brachioradialis = subject.find<tgBasicActuator>("brachioradialis");
	const std::vector<tgBasicActuator*> supportstrings  = subject.find<tgBasicActuator>("support");

    for (size_t i=0; i<olecranonfascia.size(); i++) {
		tgBasicActuator * const pMuscle = olecranonfascia[i];
		assert(pMuscle != NULL);
		pMuscle->setControlInput(olecranonfascia_length, dt);
    }
                                        
    // using for loops to anticipate more muscle fibers in the future
    for (size_t i=0; i<anconeus.size(); i++) {
		tgBasicActuator * const pMuscle = anconeus[i];
		assert(pMuscle != NULL);
		pMuscle->setControlInput(anconeus_length, dt);
    }
     
    for (size_t i=0; i<brachioradialis.size(); i++) {
		tgBasicActuator * const pMuscle = brachioradialis[i];
		assert(pMuscle != NULL);
		pMuscle->setControlInput(brachioradialis_length, dt);
    }
    
    for (size_t i=0; i<supportstrings.size(); i++) {
		tgBasicActuator * const pMuscle = supportstrings[i];
		assert(pMuscle != NULL);
		pMuscle->setControlInput(supportstring_length, dt);
        cout << "string " << i << "\n";
    }

    /*
	const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
	for (size_t i = 0; i < muscles.size(); ++i) {
		tgBasicActuator * const pMuscle = muscles[i];
		assert(pMuscle != NULL);
		pMuscle->setControlInput(this->m_initialLengths,0.0001);
	}*/
}

// Set target length of each muscle, then move motors accordingly
void ScarrArmController::onStep(ScarrArmModel& subject, double dt)
{
    // Update controller's internal time
    if (dt <= 0.0) { throw std::invalid_argument("dt is not positive"); }
    m_totalTime+=dt;

    // Set target length of each brachioradils
    const double mean_brachioradialis_length = 262/4;
    double newLength = 0;
    const std::vector<tgBasicActuator*> brachioradialis = subject.find<tgBasicActuator>("brachioradialis");
    const double amplitude    = mean_brachioradialis_length/4;
    const double angular_freq = 50; //TODO: Test for demo
    const double phase = 0;
    const double dcOffset     = mean_brachioradialis_length;

    for (size_t i=0; i<brachioradialis.size(); i++) {
		tgBasicActuator * const pMuscle = brachioradialis[i];
		assert(pMuscle != NULL);
        cout <<"t: " << pMuscle->getCurrentLength() << endl;
        newLength = amplitude * sin(angular_freq * m_totalTime + phase) + dcOffset;
        std::cout<<"calculating brachiolength:" << newLength << "\n";
		pMuscle->setControlInput(newLength, dt);
		pMuscle->moveMotors(dt);
        cout <<"t+1: " << pMuscle->getCurrentLength() << endl;
    }

/*
    //Move motors for all the muscles
	const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
	for (size_t i = 0; i < muscles.size(); ++i) {
		tgBasicActuator * const pMuscle = muscles[i];
		assert(pMuscle != NULL);
		pMuscle->moveMotors(dt);
	}
*/	
     
    /*
    for(int iMuscle=0; iMuscle < nMuscles; iMuscle++) {
        const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
        tgBasicActuator *const pMuscle = muscles[iMuscle];
        assert(pMuscle != NULL);

        double newLength = amplitude[cluster] * sin(angularFrequency[cluster] * m_totalTime + phase) + dcOffset[cluster];
        double minLength = m_initialLengths * (1-maxStringLengthFactor);
        double maxLength = m_initialLengths * (1+maxStringLengthFactor);
        if (newLength <= minLength) {
            newLength = minLength;
        } else if (newLength >= maxLength) {
            newLength = maxLength;
        }
        pMuscle->setControlInput(newLength, dt);
        if (oldCluster != cluster) {
            phase += phaseChange[cluster];
        }
    }        
    */

    /*
	//vector<double> state=getState();
	vector< vector<double> > actions;

	//get the actions (between 0 and 1) from evolution (todo)
	//actions=evolutionAdapter.step(dt,state);

	//instead, generate it here for now!
	for(unsigned i=0;i<24;i++) {
		vector<double> tmp;
		for(unsigned j=0;j<2;j++)
		{
			tmp.push_back(0.5);
		}
		actions.push_back(tmp);
	}

	//transform them to the size of the structure
	actions = transformActions(actions);

	//apply these actions to the appropriate muscles according to the sensor values
    //applyActions(subject,actions);
    */
}

//Scale actions according to Min and Max length of muscles.
vector< vector <double> > ScarrArmController::transformActions(vector< vector <double> > actions)
{
	double min=6;
	double max=11;
	double range=max-min;
	double scaledAct;
	for(unsigned i=0;i<actions.size();i++) {
		for(unsigned j=0;j<actions[i].size();j++) {
			scaledAct=actions[i][j]*(range)+min;
			actions[i][j]=scaledAct;
		}
	}
	return actions;
}

//Pick particular muscles (according to the structure's state) and apply the given actions one by one
void ScarrArmController::applyActions(ScarrArmModel& subject, vector< vector <double> > act)
{
	//Get All the muscles of the subject
	const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
	//Check if the number of the actions match the number of the muscles
	if(act.size() != muscles.size()) {
		cout<<"Warning: # of muscles: "<< muscles.size() << " != # of actions: "<< act.size()<<endl;
		return;
	}
	//Apply actions (currently in a random order)
	for (size_t i = 0; i < muscles.size(); ++i)	{
		tgBasicActuator * const pMuscle = muscles[i];
		assert(pMuscle != NULL);
		//cout<<"i: "<<i<<" length: "<<act[i][0]<<endl;
		pMuscle->setControlInput(act[i][0]);
	}
}
