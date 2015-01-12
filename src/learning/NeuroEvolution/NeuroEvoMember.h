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

#ifndef NEUROEVOMEMBER_H_
#define NEUROEVOMEMBER_H_

/**
 * @file NeuroEvoMember.h
 * @brief Single set of params for NeuroEvolution
 * @date August 2013
 * @author Atil Iscen
 * $Id$
 */

#include <string>
#include <vector>
#include <tr1/random>
#include "learning/Configuration/configuration.h"

// Forward Declarations
class neuralNetwork;

class NeuroEvoMember
{
public:
	NeuroEvoMember(configuration config);
	~NeuroEvoMember();
	void mutate(std::tr1::ranlux64_base_01 *eng);

	neuralNetwork* getNn(){
		return nn;
	}

    void copyFrom(NeuroEvoMember *otherMember);
    void copyFrom(NeuroEvoMember *otherMember1, NeuroEvoMember *otherMember2, std::tr1::ranlux64_base_01 *eng);
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
	neuralNetwork *nn;

	int numInputs;
	int numOutputs;
};



#endif /* NEUROEVOMEMBER_H_ */
