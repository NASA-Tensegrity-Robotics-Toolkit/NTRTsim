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

#ifndef NEUROADAPTER_H_
#define NEUROADAPTER_H_

/**
 * @file NeuroAdapter.h
 * @brief Defines a class NeuroAdapter to pass parameters from NeuroEvolution to a controller.
 * @date August 2013
 * @author Atil Iscen
 * $Id$
 */

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
    /** Appears unused */
	double totalTime;
};

#endif /* NEUROADAPTER_H_ */
