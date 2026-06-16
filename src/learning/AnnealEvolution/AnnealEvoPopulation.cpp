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
 * @file AnnealEvoPopulation.cpp
 * @brief Contains the implementation of class AnnealEvoPopulation
 * Adapting NeuroEvolution to do Simulated Annealing
 * @date April 2014
 * @author Brian Tietz and Atil Iscen
 * $Id$
 */

#include "AnnealEvoPopulation.h"
#include <string>
#include <vector>
#include <iostream>
#include <numeric>
#include <fstream>
#include <algorithm>

using namespace std;

AnnealEvoPopulation::AnnealEvoPopulation(int populationSize,configuration config)
{
    compareAverageScores=true;
    clearScoresBetweenGenerations=false;
    this->compareAverageScores=config.getintvalue("compareAverageScores");
    this->clearScoresBetweenGenerations=config.getintvalue("clearScoresBetweenGenerations");

    for(int i=0;i<populationSize;i++)
    {
        //cout<<"  creating members"<<endl;
        controllers.push_back(new AnnealEvoMember(config));
    }
}

AnnealEvoPopulation::~AnnealEvoPopulation()
{
    for(std::size_t i=0;i<controllers.size();i++)
    {
        delete controllers[i];
    }
}

void AnnealEvoPopulation::mutate(std::ranlux48_base *engPntr,std::size_t numMutate, double T)
{
    for(std::size_t i=0;i<numMutate;i++)
    {
        int copyFrom = 0; // Always copy from the best
        int copyTo = this->controllers.size()-1-i;
        controllers.at(copyTo)->copyFrom(controllers.at(copyFrom));
        controllers.at(copyTo)->mutate(engPntr, T);
    }
    return;
}

bool AnnealEvoPopulation::comparisonFuncForAverage(AnnealEvoMember * elm1, AnnealEvoMember * elm2)
{
        return elm1->averageScore > elm2->averageScore;
}
bool AnnealEvoPopulation::comparisonFuncForMax(AnnealEvoMember * elm1, AnnealEvoMember * elm2)
{
    return elm1->maxScore > elm2->maxScore;
}


void AnnealEvoPopulation::orderPopulation()
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
//  cout<<"ordering the whole population"<<endl;
    if(compareAverageScores)
        sort(controllers.begin(),controllers.end(),this->comparisonFuncForAverage);
    else
        sort(controllers.begin(),controllers.end(),this->comparisonFuncForMax);

}

void AnnealEvoPopulation::readConfigFromXML(std::string configFile)
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
