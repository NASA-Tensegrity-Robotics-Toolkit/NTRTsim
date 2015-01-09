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

#ifndef SRC_UTIL_CPGS_CPGEQUATIONS
#define SRC_UTIL_CPGS_CPGEQUATIONS

/**
 * @file CPGEquations.h
 * @brief Definition of class CPGEquations
 * @date March 2014
 * @author Brian Mirletz
 * $Id$
 */

#include <vector>
#include <sstream>

#include "CPGNode.h"

/**
 * The top level class for interfacing with CPGs. Contains the definition
 * of the CPG (list of nodes) as well as functions to interface with ODEInt
 */
class CPGEquations
{
 public:
	
	CPGEquations(int maxSteps = 200);

	CPGEquations(std::vector<CPGNode*>& newNodeList, int maxSteps = 200);
	
	virtual ~CPGEquations();
	
	int addNode(std::vector<double>& newParams);

	 void defineConnections (int nodeIndex,
				 std::vector<int> connections,
				 std::vector<double> newWeights,
				 std::vector<double> newPhaseOffsets);
	
	const double operator[](const std::size_t i) const;

	virtual std::vector<double>& getXVars();
	
	virtual std::vector<double>& getDXVars();
	
	virtual void updateNodes(std::vector<double>& descCom);
	
	virtual void updateNodeData(std::vector<double> newXVals);
	
	/**
	 * Call the integrator a the specified timestep
	 */
	void update(std::vector<double>& descCom, double dt);
	
	std::string toString(const std::string& prefix = "") const;
	
    void countStep()
    {
        numSteps++;
    }
    
protected:
	
	std::vector<CPGNode*> nodeList;
	
    std::vector<double> XVars;
    std::vector<double> DXVars;
    
	double stepSize;
    
    int m_maxSteps;
    int numSteps;
    
};

/**
 * Overload operator<<() to handle CPGEquations
 * @param[in,out] os an ostream
 * @param[in] a reference to a CPGEquations
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 */
inline std::ostream&
operator<<(std::ostream& os, const CPGEquations& obj)
{
    os << obj.toString() << std::endl;
	return os;
}

#endif // SIMULATOR_SRC_LIB_MODELS_SNAKE_CPGS_CPGEQUATIONS
