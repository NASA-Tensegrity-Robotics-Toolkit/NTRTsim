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
 * @file Escape_T6Controller.cpp
 * @brief Escape Controller for T6 
 * @author Steven Lessard
 * @version 1.0.0
 * $Id$
 */

// This module
#include "Escape_T6Controller.h"
// This application
#include "Escape_T6Model.h"
// This library
#include "core/tgLinearString.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>

using namespace std;

//Constructor using the model subject and a single pref length for all muscles.
//Currently calibrated to decimeters
Escape_T6Controller::Escape_T6Controller(const double initialLength)
{
    this->m_initialLengths=initialLength;
    this->m_totalTime=0.0;
}

//Fetch all the muscles and set their preferred length
void Escape_T6Controller::onSetup(Escape_T6Model& subject)
{
    const std::vector<tgLinearString*> muscles = subject.getAllMuscles();
    for (size_t i = 0; i < muscles.size(); ++i)
    {
        tgLinearString * const pMuscle = muscles[i];
        assert(pMuscle != NULL);
        pMuscle->setRestLength(this->m_initialLengths,0.0001);
    }
}

void Escape_T6Controller::onStep(Escape_T6Model& subject, double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    m_totalTime+=dt;

    //Move motors for all the muscles
    const std::vector<tgLinearString*> muscles = subject.getAllMuscles();
    for (size_t i = 0; i < muscles.size(); ++i)
    {
        tgLinearString * const pMuscle = muscles[i];
        assert(pMuscle != NULL);
        pMuscle->moveMotors(dt);
    }

    //vector<double> state=getState();
    vector< vector<double> > actions;

    //get the actions (between 0 and 1) from evolution (todo)
    //actions=evolutionAdapter.step(dt,state);

    //instead, generate it here for now!
    for(int i=0;i<24;i++)
    {
        vector<double> tmp;
        for(int j=0;j<2;j++)
        {
            tmp.push_back(0.5);
        }
        actions.push_back(tmp);
    }

    //transform them to the size of the structure
    actions = transformActions(actions);

    //apply these actions to the appropriate muscles according to the sensor values
    //	applyActions(subject,actions);

}

//Scale actions according to Min and Max length of muscles.
vector< vector <double> > Escape_T6Controller::transformActions(vector< vector <double> > actions)
{
    double min=6;
    double max=11;
    double range=max-min;
    double scaledAct;
    for(int i=0;i<actions.size();i++)
    {
        for(int j=0;j<actions[i].size();j++)
        {
            scaledAct=actions[i][j]*(range)+min;
            actions[i][j]=scaledAct;
        }
    }
    return actions;
}

//Pick particular muscles (according to the structure's state) and apply the given actions one by one
void Escape_T6Controller::applyActions(Escape_T6Model& subject, vector< vector <double> > act)
{
    //Get All the muscles of the subject
    const std::vector<tgLinearString*> muscles = subject.getAllMuscles();
    //Check if the number of the actions match the number of the muscles
    if(act.size() != muscles.size())
    {
        cout<<"Warning: # of muscles: "<< muscles.size() << " != # of actions: "<< act.size()<<endl;
        return;
    }
    //Apply actions (currently in a random order)
    for (size_t i = 0; i < muscles.size(); ++i)
    {
        tgLinearString * const pMuscle = muscles[i];
        assert(pMuscle != NULL);
        //cout<<"i: "<<i<<" length: "<<act[i][0]<<endl;
        pMuscle->setPrefLength(act[i][0]);
    }
}
