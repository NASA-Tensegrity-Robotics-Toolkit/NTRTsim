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

#ifndef NEUROEVOPOPULATION_H_
#define NEUROEVOPOPULATION_H_

/**
 * @file NeuroEvoPopulation.h
 * @brief A set of members that gets updated and mutated
 * @date August 2013
 * @author Atil Iscen
 * $Id$
 */

#include "NeuroEvoMember.h"
#include <vector>
#include <tr1/random>

class NeuroEvoPopulation {
public:
	NeuroEvoPopulation(int numControllers, configuration& config);
	~NeuroEvoPopulation();
	std::vector<NeuroEvoMember *> controllers;
    void mutate(std::tr1::ranlux64_base_01 *eng,std::size_t numToMutate);
	void combineAndMutate(std::tr1::ranlux64_base_01 *eng, std::size_t numToMutate, std::size_t numToCombine);
	void orderPopulation();
	NeuroEvoMember * getMember(int i){return controllers[i];};

private:
    std::vector<double> generateMatingProbabilities();
    int getIndexFromProbability(std::vector<double>& probs, double val);
	static bool comparisonFuncForAverage(NeuroEvoMember * elm1, NeuroEvoMember * elm2);
	static bool comparisonFuncForMax(NeuroEvoMember * elm1, NeuroEvoMember * elm2);
	bool compareAverageScores;
	bool clearScoresBetweenGenerations;
	int populationSize;
    configuration m_config;
};



#endif /* NEUROEVOPOPULATION_H_ */
