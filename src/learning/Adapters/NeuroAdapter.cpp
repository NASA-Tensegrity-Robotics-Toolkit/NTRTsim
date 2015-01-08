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
 * @file NeuroAdapter.cpp
 * @brief Defines a class NeuroAdapter to pass parameters from NeuroEvolution to a controller.
 * @date August 2013
 * @author Atil Iscen
 * $Id$
 */

#include "NeuroAdapter.h"
#include "learning/Configuration/configuration.h"
#include "helpers/FileHelpers.h"
#include "neuralNet/Neural Network v2/neuralNetwork.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <assert.h>

using namespace std;

NeuroAdapter::NeuroAdapter() :
totalTime(0.0)
{
}
NeuroAdapter::~NeuroAdapter(){};

void NeuroAdapter::initialize(NeuroEvolution *evo,bool isLearning,configuration configdata)
{
	numberOfActions=configdata.getDoubleValue("numberOfActions");
	numberOfStates=configdata.getDoubleValue("numberOfStates");
	numberOfControllers=configdata.getDoubleValue("numberOfControllers");
	totalTime=0.0;

	//This Function initializes the parameterset from evo.
	this->neuroEvo = evo;
	if(isLearning)
	{
		currentControllers = this->neuroEvo->nextSetOfControllers();
	}
	else
	{
		currentControllers = this->neuroEvo->nextSetOfControllers();
		for(std::size_t i=0;i<currentControllers.size();i++)
		{
			stringstream ss;
			ss<< neuroEvo->resourcePath << "logs/bestParameters-"<<this->neuroEvo->suffix<<"-"<<i<<".nnw";
			currentControllers[i]->loadFromFile(ss.str().c_str());
		}
	}
	errorOfFirstController=0.0;
}

vector<vector<double> > NeuroAdapter::step(double deltaTimeSeconds,vector<double> state)
{
	totalTime+=deltaTimeSeconds;
//	cout<<"NN adapter, state: "<<state[0]<<" "<<state[1]<<" "<<state[2]<<" "<<state[3]<<" "<<state[4]<<" "<<endl;
	vector< vector<double> > actions;
	if(numberOfStates>0)
	{
		double *inputs = new double[numberOfStates];

		//scale inputs to 0-1 from -1 to 1 (unit vector provided from the controller).
		// Assumes inputs are already scaled -1 to 1
		assert (state.size() == numberOfStates);
		for (int i = 0; i < numberOfStates; i++)
		{
			inputs[i]=state[i] / 2.0 + 0.5;
		}
		for(std::size_t i=0;i<currentControllers.size();i++)
		{
			double *output=currentControllers[i]->getNn()->feedForwardPattern(inputs);
			vector<double> tmpAct;
			for(int j=0;j<numberOfActions;j++)
			{
				tmpAct.push_back(output[j]);
			}
			actions.push_back(tmpAct);
		}
		delete[]inputs;
	}
	else
	{
		for(std::size_t i=0;i<currentControllers.size();i++)
		{
			vector<double> tmpAct;
			for(std::size_t j=0;j<currentControllers[i]->statelessParameters.size();j++)
			{
				tmpAct.push_back(currentControllers[i]->statelessParameters[j]);
			}
			actions.push_back(tmpAct);
		}
	}


    return actions;
}

void NeuroAdapter::endEpisode(vector<double> scores)
{
	if(scores.size()==0)
	{
		vector< double > tmp(1);
		tmp[0]=-1;
		neuroEvo->updateScores(tmp);
		cout<<"Exploded"<<endl;
	}
	else
	{
		cout<<"Dist Moved: "<<scores[0]<<" energy: "<<scores[1]<<endl;
//		double combinedScore=scores[0]*1.0-scores[1]*1.0;
		neuroEvo->updateScores(scores);
	}
	return;
}
