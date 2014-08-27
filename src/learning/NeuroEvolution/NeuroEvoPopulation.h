/*
 * neuroEvoPopulation.h
 *
 *  Created on: Aug 5, 2013
 *      Author: atiliscen
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
	void mutate(std::tr1::ranlux64_base_01 *eng,int numToMutate);
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
