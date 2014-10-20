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
 * @file AnnealAdapter.cpp
 * @brief Contains the implementation of class AnnealAdapter.
 * Adapting NeuroEvolution to do Simulated Annealing
 * @date April 2014
 * @author Brian Tietz and Atil Iscen
 * $Id$
 */

#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include "AnnealAdapter.h"
#include "learning/Configuration/configuration.h"
#include "helpers/FileHelpers.h"

using namespace std;

AnnealAdapter::AnnealAdapter() :
totalTime(0.0)
{
}
AnnealAdapter::~AnnealAdapter(){};

void AnnealAdapter::initialize(AnnealEvolution *evo,bool isLearning,configuration configdata)
{
    numberOfActions=configdata.getDoubleValue("numberOfActions");
    numberOfStates=configdata.getDoubleValue("numberOfStates");
    numberOfControllers=configdata.getDoubleValue("numberOfControllers");
    totalTime=0.0;

    //This Function initializes the parameterset from evo.
    this->annealEvo = evo;
    if(isLearning)
    {
        currentControllers = this->annealEvo->nextSetOfControllers();
    }
    else
    {
        currentControllers = this->annealEvo->nextSetOfControllers();
        for(int i=0;i<currentControllers.size();i++)
        {
            stringstream ss;
            ss << annealEvo->resourcePath << "logs/bestParameters-" << this->annealEvo->suffix << "-" << i << ".nnw";
            currentControllers[i]->loadFromFile(ss.str().c_str());
//          currentControllers[i]->getNn()->loadWeights(ss.str().c_str());
        }
    }
    errorOfFirstController=0.0;
}

vector<vector<double> > AnnealAdapter::step(double deltaTimeSeconds,vector<double> state)
{
    totalTime+=deltaTimeSeconds;
//  cout<<"NN adapter, state: "<<state[0]<<" "<<state[1]<<" "<<state[2]<<" "<<state[3]<<" "<<state[4]<<" "<<endl;
    vector< vector<double> > actions;

    for(int i=0;i<currentControllers.size();i++)
    {
        vector<double> tmpAct;
        for(int j=0;j<currentControllers[i]->statelessParameters.size();j++)
        {
            tmpAct.push_back(currentControllers[i]->statelessParameters[j]);
        }
        actions.push_back(tmpAct);
    }

    return actions;
}

void AnnealAdapter::endEpisode(vector<double> scores)
{
    if(scores.size()==0)
    {
        vector< double > tmp(1);
        tmp[0]=-1;
        annealEvo->updateScores(tmp);
        cout<<"Exploded"<<endl;
    }
    else
    {
        cout<<"Dist Moved: "<<scores[0]<<" energy: "<<scores[1]<<endl;
//      double combinedScore=scores[0]*1.0-scores[1]*1.0;
        annealEvo->updateScores(scores);
    }
    return;
}
