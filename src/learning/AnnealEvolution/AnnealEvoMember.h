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

#ifndef ANNEALEVOMEMBER_H_
#define ANNEALEVOMEMBER_H_

/**
 * @file AnnealEvoMember.h
 * @brief Contains the definition of class AnnealEvoMember
 * Adapting NeuroEvolution to do Simulated Annealing
 * @date April 2014
 * @author Brian Tietz and Atil Iscen
 * $Id$
 */

#include <string>
#include <vector>
#include <tr1/random>
#include "learning/Configuration/configuration.h"


class AnnealEvoMember
{
public:
    AnnealEvoMember(configuration config);
    ~AnnealEvoMember();
    void mutate(std::tr1::ranlux64_base_01 *eng, double T);

    void copyFrom(AnnealEvoMember *otherMember);
    void saveToFile(const char* outputFilename);
    void loadFromFile(const char* inputFilename);

    std::vector<double> statelessParameters;
    //scores for evaluation
    std::vector<double> pastScores;
    double maxScore;
    double maxScore1;
    double maxScore2;
    double averageScore;

private:
    int numOutputs;
    double devBase;
    bool monteCarlo;
};



#endif /* ANNEALEVOMEMBER_H_ */
