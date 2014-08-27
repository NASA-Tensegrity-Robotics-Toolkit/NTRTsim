/*
 * NeuroEvolution.h
 *
 *  Created on: Aug 5, 2013
 *      Author: atiliscen
 */

#ifndef NEUROEVOLUTION_H_
#define NEUROEVOLUTION_H_

#include "NeuroEvoPopulation.h"
#include "NeuroEvoMember.h"
#include <fstream>

class NeuroEvolution
{
public:
	NeuroEvolution(string suffix, string config = "config.ini");
	~NeuroEvolution();
	void mutateEveryController();
	void orderAllPopulations();
	void evaluatePopulation();
	vector< NeuroEvoMember *> nextSetOfControllers();
	void updateScores(vector<double> scores);
	string suffix;
private:
	int populationSize;
	int numberOfControllers;
	std::tr1::ranlux64_base_01 eng;
	vector< NeuroEvoPopulation *> populations;
	vector <NeuroEvoMember *>  selectedControllers;
	vector< vector< double > > scoresOfTheGeneration;
//	double minValue;
//	double maxValue;
	double leniencyCoef;
	bool coevolution;
	ofstream evolutionLog;
	int currentTest;
	int numberOfTestsBetweenGenerations;
	int generationNumber;
	int numberOfElementsToMutate;
};

#endif /* NEUROEVOLUTION_H_ */
