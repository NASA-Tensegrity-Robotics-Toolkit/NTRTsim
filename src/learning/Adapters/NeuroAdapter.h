/*
 * NeuroAdapter.h
 *
 *  Created on: Aug 5, 2013
 *      Author: atiliscen
 */

#ifndef NEUROADAPTER_H_
#define NEUROADAPTER_H_

#include <vector>
#include "../NeuroEvolution/NeuroEvolution.h"
#include "../NeuroEvolution/NeuroEvoMember.h"

class NeuroAdapter
{
public:
	NeuroAdapter();
	~NeuroAdapter();
	/**
	 * Initialize needs to be called at the beginning of each trial
	 * For NTRT this means main or simulator needs to own the pointer to
	 * NeuroEvolution, we can't create it here
	 */
	void initialize(NeuroEvolution *evo,bool isLearning,configuration config);
	std::vector<std::vector<double> > step(double deltaTimeSeconds, std::vector<double> state);
	void endEpisode(std::vector<double> state);

private:
	int numberOfActions;
	int numberOfStates;
	int numberOfControllers;
	NeuroEvolution *neuroEvo;
	std::vector< NeuroEvoMember *>currentControllers;
	std::vector<double> initialPosition;
	double errorOfFirstController;
	double totalTime;
};

#endif /* NEUROADAPTER_H_ */
