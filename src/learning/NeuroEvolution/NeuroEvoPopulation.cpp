/*
 * NeuroEvoPopulation.cpp
 *
 *  Created on: Aug 5, 2013
 *      Author: atiliscen
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
	for(int i=0;i<controllers.size();i++)
	{
		delete controllers[i];
	}
}

void NeuroEvoPopulation::mutate(std::tr1::ranlux64_base_01 *engPntr,int numMutate)
{
	if(numMutate>controllers.size()/2)
	{
		cout<<"Trying to mutate more than half of the population"<<endl;
		exit(0);
	}
	for(int i=0;i<numMutate;i++)
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
	for(int i=0;i<this->controllers.size();i++)
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

void NeuroEvoPopulation::readConfigFromXML(string configFile)
{
	int intValue;
	string elementTxt;
	ifstream in;
	in.open(configFile.c_str());
	if(in.fail()==true)
	{
		cerr << endl << "impossible to read file " << configFile << endl;
		in.close();
		exit(1);
		return;
	}
	do
	{
		in >> ws >> elementTxt;
		if(elementTxt == "<!--")
		{
			do
			{
				in >> ws >> elementTxt;
			}while(elementTxt != "-->");
		}
		else if(elementTxt == "compareAverageScores")
		{
			in >> ws >> intValue;
			this->compareAverageScores=intValue;
		}
		else if(elementTxt == "clearScoresBetweenGenerations")
		{
			in >> ws >> intValue;
			this->clearScoresBetweenGenerations=intValue;
		}
		else if(elementTxt == "populationSize")
		{
			in >> ws >> intValue;
			this->populationSize=intValue;
		}
	}while(elementTxt != "</configuration>" || in.eof());
	in.close();
}
