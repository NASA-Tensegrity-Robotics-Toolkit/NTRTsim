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
// The C++ Standard Library
#include <string>
#include <vector>
#include <iostream>
#include <numeric>
#include <fstream>
#include <algorithm>
#include <stdexcept>
#include <cassert>

using namespace std;

NeuroEvoPopulation::NeuroEvoPopulation(int populationSize,configuration& config) :
m_config(config),
compareAverageScores(true),
clearScoresBetweenGenerations(false)
{
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

void NeuroEvoPopulation::mutate(std::ranlux48_base *engPntr,std::size_t numMutate)
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

void NeuroEvoPopulation::combineAndMutate(std::ranlux48_base *eng, std::size_t numToMutate, std::size_t numToCombine)
{
    std::uniform_real_distribution<double> unif(0, 1);
    
    if(numToMutate + numToCombine > controllers.size())
    {
        throw std::invalid_argument("Population will grow in size with these parameters");
    }
    
    std::vector<double> probabilities = generateMatingProbabilities();
    
    std::vector<NeuroEvoMember*> newControllers;
    for(int i = 0; i < numToCombine; i++)
    {
        
        double val1 = unif(*eng);
        double val2 = unif(*eng);
        
        int index1 = getIndexFromProbability(probabilities, val1);
        int index2 = getIndexFromProbability(probabilities, val2);
        
        if(index1 == index2)
        {
            if(index2 == 0)
            {
                index2++;
            }
            else
            {
                index2--;
            }
        }
        
        NeuroEvoMember* newController = new NeuroEvoMember(m_config);
        newController->copyFrom(controllers[index1], controllers[index2], eng);
        
        if(unif(*eng) > 0.9)
        {
            newController->mutate(eng);
        }
        
        newControllers.push_back(newController);
    }
    
    for(int i = 0; i < numToMutate; i++)
    {
        double val1 = unif(*eng);
        int index1 = getIndexFromProbability(probabilities, val1);
        NeuroEvoMember* newController = new NeuroEvoMember(m_config);
        newController->copyFrom(controllers[index1]);
        newController->mutate(eng);
        newControllers.push_back(newController);
    }
    
    
    const std::size_t n = controllers.size();
    const std::size_t m = newControllers.size();
    while(controllers.size() > n - m)
    {
        controllers.pop_back();
    }
    
    controllers.insert(controllers.end(), newControllers.begin(), newControllers.end());
    
    assert(controllers.size() == n);
}


void NeuroEvoPopulation::orderPopulation()
{
	//calculate each member's average score
	for(std::size_t i=0;i<this->controllers.size();i++)
	{
		double ave = std::accumulate(controllers[i]->pastScores.begin(),controllers[i]->pastScores.end(),0);
		
        double n = (double) controllers[i]->pastScores.size(); 
        if (n > 0)
        {
            ave /= n;
        }
        else
        {
            ave = -100000;
        }
        
        //assert(controllers[i]->pastScores.size() > 0);
        
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

std::vector<double> NeuroEvoPopulation::generateMatingProbabilities()
{
    double totalScore = 0.0;
    const std::size_t n = controllers.size();
    std::vector<double> probabilties;
    if (compareAverageScores)
    {
        double floor = controllers[n - 1]->averageScore;
        
        for (std::size_t i = 0; i < n; i++)
        {
            totalScore += (controllers[i]->averageScore - floor);
        }
        
        probabilties.push_back((controllers[0]->averageScore - floor) / totalScore);
        for (std::size_t i = 1; i < n; i++)
        {
            double nextScore = (controllers[i]->averageScore - floor) / totalScore + probabilties[i - 1];
            probabilties.push_back(nextScore);
        }
    }
    else
    {
        double floor = controllers[n - 1]->maxScore;
        
        for (std::size_t i = 0; i < n; i++)
        {
            totalScore += (controllers[i]->maxScore - floor);
        }
        
        probabilties.push_back((controllers[0]->maxScore - floor) / totalScore);
        for (std::size_t i = 1; i < n; i++)
        {
            double nextScore = (controllers[i]->maxScore - floor) / totalScore + probabilties[i - 1];
            probabilties.push_back(nextScore);
        }
    }
    
    return probabilties;
}

int NeuroEvoPopulation::getIndexFromProbability(std::vector<double>& probs, double val)
{
    int i = 0;
    
    while(i < (probs.size() - 1) && probs[i] < val)
    {
        i++;
    }
    
    return i;
}

bool NeuroEvoPopulation::comparisonFuncForAverage(NeuroEvoMember * elm1, NeuroEvoMember * elm2)
{
    return elm1->averageScore > elm2->averageScore;
}
bool NeuroEvoPopulation::comparisonFuncForMax(NeuroEvoMember * elm1, NeuroEvoMember * elm2)
{
    return elm1->maxScore > elm2->maxScore;
}
