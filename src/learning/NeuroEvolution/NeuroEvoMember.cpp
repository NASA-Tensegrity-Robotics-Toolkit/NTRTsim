/*
 * NeuroEvoMember.cpp
 *
 *  Created on: Aug 5, 2013
 *      Author: atiliscen
 */


#include "NeuroEvoMember.h"
#include <fstream>
#include <iostream>

using namespace std;

NeuroEvoMember::NeuroEvoMember(configuration config)
{
	//readConfigFromXML(configFile);
	this->numInputs=config.getintvalue("numberOfStates");
	this->numOutputs=config.getintvalue("numberOfActions");

	cout<<"creating NN"<<endl;
	if(numInputs>0)
		nn = new neuralNetwork(numInputs,numInputs*2,numOutputs);
	else
	{
		statelessParameters.resize(numOutputs);
		for(int i=0;i<numOutputs;i++)
			statelessParameters[i]=rand()*1.0/RAND_MAX;
	}
	maxScore=-1000;
}

NeuroEvoMember::~NeuroEvoMember()
{
	delete nn;
}

void NeuroEvoMember::mutate(std::tr1::ranlux64_base_01 *eng){
	std::tr1::uniform_real<double> unif(0, 1);
	if(unif(*eng)  > 0.5)
	{
		return;
	}
	//TODO: for each weight of the NN with 0.5 probability mutate it

	if(numInputs>0)
		this->nn->mutate(eng);
	else
	{
		double dev = 3.0 / 100.0;   // 10 percent of interval 0-1
		std::tr1::normal_distribution<double> normal(0, dev);
		for(int i=0;i<statelessParameters.size();i++)
		{
			if(unif(*eng)  > 0.5)
			{
				continue;
			}
			double mutAmount = normal(*eng);
//			cout<<"param: "<<i<<" dev: "<<dev<<" rand: "<<mutAmount<<endl;
			double newParam= statelessParameters[i] + mutAmount;
			if(newParam < 0.0)
				statelessParameters[i] = 0.0;
			else if(newParam > 1.0)
				statelessParameters[i] = 1.0;
			else
				statelessParameters[i] =newParam;
		}

	}
}

void NeuroEvoMember::copyFrom(NeuroEvoMember* otherMember)
{
	if(numInputs>0)
	{
		this->nn->copyWeightFrom(otherMember->getNn());
		this->maxScore=-10000;
		this->pastScores.clear();
	}
	else
	{
		this->statelessParameters=otherMember->statelessParameters;
	}
}

void NeuroEvoMember::saveToFile(const char * outputFilename)
{
	if(numInputs > 0 )
		this->getNn()->saveWeights(outputFilename);
	else
	{
		ofstream ss(outputFilename);
		for(int i=0;i<statelessParameters.size();i++)
		{
			ss<<statelessParameters[i];
			if(i!=statelessParameters.size()-1)
				ss<<",";
		}
		ss.close();
	}
}

void NeuroEvoMember::loadFromFile(const char * outputFilename)
{
	if(numInputs > 0 )
		this->getNn()->loadWeights(outputFilename);
	else
	{
		//cout<<"loading parameters from file "<<outputFilename<<endl;
		ifstream ss(outputFilename);
		int i=0;
		string value;
#if (0)
		// Disable definition of unused variable to suppress compiler warning
		double valueDbl;
#endif
		while(!ss.eof())
		{
			//cout<<"success opening file"<<endl;
			if(getline ( ss, value, ',' )>0)
			{
				//cout<<"value read as string: "<<value<<endl;
				statelessParameters[i++]=atof(value.c_str());
				//cout<<statelessParameters[i-1]<<",";
			}
		}
		//cout<<"reading complete"<<endl;
		cout<<endl;
		ss.close();
	}
}
