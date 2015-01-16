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

#ifndef NEUROEVOLUTION_H_
#define NEUROEVOLUTION_H_
/**
 * @file NeuroEvolution.h
 * @brief Top level class for NeuroEvolution
 * @date August 2013
 * @author Atil Iscen
 * $Id$
 */



#include "NeuroEvoPopulation.h"
#include "NeuroEvoMember.h"
#include <fstream>

class NeuroEvolution
{
public:
	NeuroEvolution(std::string suffix, std::string config = "config.ini", std::string path = "");
	~NeuroEvolution();
	void mutateEveryController();
    void combineAndMutate();
	void orderAllPopulations();
	void evaluatePopulation();
	std::vector< NeuroEvoMember *> nextSetOfControllers();
	void updateScores(std::vector<double> scores);
    const std::string suffix;
    /// @todo make this const if we decide to force everyone to put their logs in resources
    std::string resourcePath;
private:
	int populationSize;
	int numberOfControllers;
	std::tr1::ranlux64_base_01 eng;
	std::vector< NeuroEvoPopulation *> populations;
	std::vector <NeuroEvoMember *>  selectedControllers;
	std::vector< std::vector< double > > scoresOfTheGeneration;
//	double minValue;
//	double maxValue;
	double leniencyCoef;
    bool seeded;
	bool coevolution;
	std::ofstream evolutionLog;
	int currentTest;
	int numberOfTestsBetweenGenerations;
	int generationNumber;
	int numberOfElementsToMutate;
    int numberOfChildren;
    int numberOfSubtests;
    int subTests;
};

#endif /* NEUROEVOLUTION_H_ */
