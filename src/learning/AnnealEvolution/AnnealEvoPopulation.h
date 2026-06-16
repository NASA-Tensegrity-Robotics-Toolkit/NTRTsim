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

#ifndef ANNEALEVOPOPULATION_H_
#define ANNEALEVOPOPULATION_H_

/**
 * @file AnnealEvoPopulation.h
 * @brief Contains the definition of class AnnealEvoPopulation
 * Adapting NeuroEvolution to do Simulated Annealing
 * @date April 2014
 * @author Brian Tietz and Atil Iscen
 * $Id$
 */

#include "AnnealEvoMember.h"
#include <vector>

class AnnealEvoPopulation {
public:
    AnnealEvoPopulation(int numControllers,configuration config);
    ~AnnealEvoPopulation();
    std::vector<AnnealEvoMember *> controllers;
    void mutate(std::ranlux48_base *eng,std::size_t numToMutate, double T);
    void orderPopulation();
    AnnealEvoMember * selectMemberToEvaluate();
    AnnealEvoMember * getMember(int i){return controllers[i];};

private:
    static bool comparisonFuncForAverage(AnnealEvoMember * elm1, AnnealEvoMember * elm2);
    static bool comparisonFuncForMax(AnnealEvoMember * elm1, AnnealEvoMember * elm2);
    void readConfigFromXML(std::string configFile);
    bool compareAverageScores;
    bool clearScoresBetweenGenerations;
    int populationSize;
};



#endif /* ANNEALEVOPOPULATION_H_ */
