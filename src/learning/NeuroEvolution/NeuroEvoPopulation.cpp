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
 * @file NeuroEvoPopulation.cpp
 * @brief A set of members that gets updated and mutated
 * @date August 2013
 * @author Atil Iscen
 * $Id$
 */

#include "NeuroEvoPopulation.h"
#include <string>
#include <vector>
#include <iostream>
#include <numeric>
#include <fstream>
#include <algorithm>

using namespace std;

NeuroEvoPopulation::NeuroEvoPopulation(int populationSize,configuration config)
{
	compareAverageScores=true;
	clearScoresBetweenGenerations=false;
	this->compareAverageScores=config.getintvalue("compareAverageScores");
	this->clearScoresBetweenGenerations=config.getintvalue("clearScoresBetweenGenerations");

	for(int i=0;i<populationSize;i++)
	{
		cout<<"  creating members"<<endl;
		controllers.push_back(new NeuroEvoMember(config));
	}
}

NeuroEvoPopulation::~NeuroEvoPopulation()
{
	for(std::size_t i=0;i<controllers.size();i++)
	{
		delete controllers[i];
	}
}

void NeuroEvoPopulation::mutate(std::tr1::ranlux64_base_01 *engPntr,std::size_t numMutate)
{
	if(numMutate>controllers.size()/2)
	{
		cout<<"Trying to mutate more than half of the population"<<endl;
		exit(0);
	}
	for(std::size_t i=0;i<numMutate;i++)
	{
		int copyFrom = i;
		int copyTo = this->controllers.size()-1-i;
		controllers.at(copyTo)->copyFrom(controllers.at(copyFrom));
		controllers.at(copyTo)->mutate(engPntr);
	}
	return;
}

bool NeuroEvoPopulation::comparisonFuncForAverage(NeuroEvoMember * elm1, NeuroEvoMember * elm2)
{
		return elm1->averageScore > elm2->averageScore;
}
bool NeuroEvoPopulation::comparisonFuncForMax(NeuroEvoMember * elm1, NeuroEvoMember * elm2)
{
	return elm1->maxScore > elm2->maxScore;
}


void NeuroEvoPopulation::orderPopulation()
{
	//calculate each member's average score
	for(std::size_t i=0;i<this->controllers.size();i++)
	{
		double ave = std::accumulate(controllers[i]->pastScores.begin(),controllers[i]->pastScores.end(),0);
		ave /=  (double) controllers[i]->pastScores.size();
		controllers[i]->averageScore=ave;
		if(clearScoresBetweenGenerations)
			controllers[i]->pastScores.clear();
	}
//	cout<<"ordering the whole population"<<endl;
	if(compareAverageScores)
		sort(controllers.begin(),controllers.end(),this->comparisonFuncForAverage);
	else
		sort(controllers.begin(),controllers.end(),this->comparisonFuncForMax);

}
