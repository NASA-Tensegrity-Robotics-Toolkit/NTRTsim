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
 * @file NeuroEvoMember.cpp
 * @brief Single set of params for NeuroEvolution
 * @date August 2013
 * @author Atil Iscen
 * $Id$
 */

#include "NeuroEvoMember.h"
#include "neuralNet/Neural Network v2/neuralNetwork.h"
#include <fstream>
#include <iostream>
#include <assert.h>
#include <stdexcept>

using namespace std;

NeuroEvoMember::NeuroEvoMember(configuration config)
{
	this->numInputs=config.getintvalue("numberOfStates");
    this->numOutputs=config.getintvalue("numberOfActions");
	int numHidden = config.getintvalue("numberHidden");
    assert(numOutputs > 0);
	cout<<"creating NN"<<endl;
	if(numInputs>0)
		nn = new neuralNetwork(numInputs, numHidden,numOutputs);
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
		for(std::size_t i=0;i<statelessParameters.size();i++)
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

void NeuroEvoMember::copyFrom(NeuroEvoMember *otherMember1, NeuroEvoMember *otherMember2, std::tr1::ranlux64_base_01 *eng)
{
    if(numInputs>0)
    {
        this->nn->combineWeights(otherMember1->getNn(), otherMember2->getNn(), eng);
        this->maxScore=-10000;
        this->pastScores.clear();
    }
    else
    {
        std::tr1::uniform_real<double> unif(0, 1);
        for (int i = 0; i < numOutputs; i++)
        {
            if (unif(*eng) > 0.5)
            {
                this->statelessParameters[i] = otherMember1->statelessParameters[i];
            }
            else
            {
                this->statelessParameters[i] = otherMember2->statelessParameters[i];
            }
        }
    }    
}

void NeuroEvoMember::saveToFile(const char * outputFilename)
{
	if(numInputs > 0 )
		this->getNn()->saveWeights(outputFilename);
	else
	{
		ofstream ss(outputFilename);
		for(std::size_t i=0;i<statelessParameters.size();i++)
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
		if(ss.is_open())
		{
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
		else
		{
			cout << "File of name " << outputFilename << " does not exist" << std::endl;
			cout << "Try turning learning on in config.ini to generate parameters" << std::endl;
			throw std::invalid_argument("Parameter file does not exist");
		}
		//cout<<"reading complete"<<endl;
		//cout<<endl;
		ss.close();
	}

}
