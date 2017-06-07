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

#ifndef CPG_FEEDBACK_CPGEQUATIONS_FB
#define CPG_FEEDBACK_CPGEQUATIONS_FB

/**
 * @file CPGEquationsFB.h
 * @brief Definition of class CPGEquationsFB
 * @date December 2014
 * @author Brian Mirletz
 * $Id$
 */

#include "util/CPGEquations.h"

#include "CPGNodeFB.h"


#include <vector>
#include <assert.h>
#include <sstream>


/**
 * The top level class for interfacing with CPGs. Contains the definition
 * of the CPG (list of nodes) as well as functions to interface with ODEInt
 */
class CPGEquationsFB : public CPGEquations
{
 public:
	
	CPGEquationsFB(int maxSteps = 200);

	CPGEquationsFB(std::vector<CPGNode*>& newNodeList, int maxSteps = 200);
	
	~CPGEquationsFB();
	
    int addNode(std::vector<double>& newParams);
    
	std::vector<double>& getXVars();
	
	std::vector<double>& getDXVars();
	
	void updateNodes(std::vector<double>& descCom);
	
	void updateNodeData(std::vector<double> newXVals);

};

#endif // SIMULATOR_SRC_LIB_MODELS_SNAKE_CPGS_CPGEQUATIONS
