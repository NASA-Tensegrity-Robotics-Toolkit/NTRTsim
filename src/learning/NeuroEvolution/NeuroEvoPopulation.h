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
 * @file NeuroEvoPopulation.h
 * @brief A set of members that gets updated and mutated
 * @date August 2013
 * @author Atil Iscen
 * $Id$
 */

#ifndef NEUROEVOPOPULATION_H_
#define NEUROEVOPOPULATION_H_

#include "NeuroEvoMember.h"
#include <vector>

class NeuroEvoPopulation {
public:
	NeuroEvoPopulation(int numControllers,configuration config);
	~NeuroEvoPopulation();
	vector<NeuroEvoMember *> controllers;
	void mutate(std::tr1::ranlux64_base_01 *eng,std::size_t numToMutate);
	void orderPopulation();
	NeuroEvoMember * selectMemberToEvaluate();
	NeuroEvoMember * getMember(int i){return controllers[i];};

private:
	static bool comparisonFuncForAverage(NeuroEvoMember * elm1, NeuroEvoMember * elm2);
	static bool comparisonFuncForMax(NeuroEvoMember * elm1, NeuroEvoMember * elm2);
	void readConfigFromXML(string configFile);
	bool compareAverageScores;
	bool clearScoresBetweenGenerations;
	int populationSize;
};



#endif /* NEUROEVOPOPULATION_H_ */
