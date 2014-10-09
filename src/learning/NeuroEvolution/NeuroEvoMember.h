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
 * @file NeuroEvoMember.h
 * @brief Single set of params for NeuroEvolution
 * @date August 2013
 * @author Atil Iscen
 * $Id$
 */

#ifndef NEUROEVOMEMBER_H_
#define NEUROEVOMEMBER_H_

#include <string>
#include <vector>
#include <tr1/random>
#include "neuralNet/Neural Network v2/neuralNetwork.h"
#include "../Configuration/configuration.h"

using namespace std;

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
	void saveToFile(const char* outputFilename);
	void loadFromFile(const char* inputFilename);

	vector<double> statelessParameters;
	//scores for evaluation
	vector<double> pastScores;
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
