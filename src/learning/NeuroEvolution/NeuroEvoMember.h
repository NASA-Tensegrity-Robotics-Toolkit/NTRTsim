/*
 * NeuroEvoMember.h
 *
 *  Created on: Aug 5, 2013
 *      Author: atiliscen
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
