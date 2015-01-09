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
 * @file AnnealEvoMember.cpp
 * @brief Contains the implementation of class AnnealEvoMember
 * Adapting NeuroEvolution to do Simulated Annealing
 * @date April 2014
 * @author Brian Tietz and Atil Iscen
 * $Id$
 */

#include "AnnealEvoMember.h"
#include <fstream>
#include <iostream>
#include <assert.h>
#include <stdexcept>

using namespace std;

AnnealEvoMember::AnnealEvoMember(configuration config)
{
    //readConfigFromXML(configFile);
    this->numOutputs=config.getintvalue("numberOfActions");
    this->devBase=config.getDoubleValue("deviation");
    this->monteCarlo=config.getintvalue("MonteCarlo");
    
    statelessParameters.resize(numOutputs);
    for(int i=0;i<numOutputs;i++)
        statelessParameters[i]=rand()*1.0/RAND_MAX;

    maxScore=-1000;
}

AnnealEvoMember::~AnnealEvoMember()
{
}

void AnnealEvoMember::mutate(std::tr1::ranlux64_base_01 *eng, double T){
    
    assert (T <= 1.0);
    std::tr1::uniform_real<double> unif(0, 1);

    //TODO: for each weight of the NN with 0.5 probability mutate it

    double dev = devBase * T / 100.0; 
    std::tr1::normal_distribution<double> normal(0, dev);
    for(std::size_t i=0;i<statelessParameters.size();i++)
    {
        double newParam;
        if (monteCarlo)
        {
            newParam= unif(*eng);
        }
        else
        {   
            double mutAmount = normal(*eng);
              //std::cout<<"param: "<<i<<" dev: "<<dev<<" rand: "<<mutAmount<<endl;
            newParam= statelessParameters[i] + mutAmount;
        }

        if(newParam < 0.0)
            statelessParameters[i] = 0.0;
        else if(newParam > 1.0)
            statelessParameters[i] = 1.0;
        else
            statelessParameters[i] =newParam;
    }


}

void AnnealEvoMember::copyFrom(AnnealEvoMember* otherMember)
{

     this->statelessParameters=otherMember->statelessParameters;

}

void AnnealEvoMember::saveToFile(const char * outputFilename)
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

void AnnealEvoMember::loadFromFile(const char * outputFilename)
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
			// @todo fix the infinite loop that occurs here!
			if(getline ( ss, value, ',' )>0)
			{
				//cout<<"value read as string: "<<value<<endl;
				statelessParameters[i++]=atof(value.c_str());
				//cout<<statelessParameters[i-1]<<",";
			}
		}
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
