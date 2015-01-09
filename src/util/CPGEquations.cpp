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
 * @file CPGEquations.cpp
 * @brief Implementation of class CPGEquations
 * @date March 2014
 * @author Brian Mirletz
 * $Id$
 */

#include "CPGEquations.h"

#include "boost/array.hpp"
#include "boost/numeric/odeint.hpp"

// The Bullet Physics Library
#include "LinearMath/btQuickprof.h"


// The C++ Standard Library
#include <assert.h>
#include <stdexcept>

using namespace boost::numeric::odeint;

typedef std::vector<double > cpgVars_type;

CPGEquations::CPGEquations(int maxSteps) :
stepSize(0.1),
numSteps(0),
m_maxSteps(maxSteps)
 {}
CPGEquations::CPGEquations(std::vector<CPGNode*>& newNodeList, int maxSteps) :
nodeList(newNodeList),
stepSize(0.1), //TODO: specify as a parameter somewhere
numSteps(0),
m_maxSteps(maxSteps)
{
}

CPGEquations::~CPGEquations()
{
	for (std::size_t i = 0; i < nodeList.size(); i++)
	{
		delete nodeList[i];
	}
	nodeList.clear();
}

// Params needs size 7 to fill all of the params.
// TODO: consider changing to a config struct
int CPGEquations::addNode(std::vector<double>& newParams) 
{
	int index = nodeList.size();
	CPGNode* newNode = new CPGNode(index, newParams);
	nodeList.push_back(newNode);
	
	return index;
}

void CPGEquations::defineConnections (	int nodeIndex,
								std::vector<int> connections,
								std::vector<double> newWeights,
								std::vector<double> newPhaseOffsets)
{
	assert(connections.size() == newWeights.size());
	assert(connections.size() == newPhaseOffsets.size());
	assert(nodeList[nodeIndex] != NULL);
	
	for(int i = 0; i != connections.size(); i++){
		nodeList[nodeIndex]->addCoupling(nodeList[connections[i]], newWeights[i], newPhaseOffsets[i]); 
	}
}

const double CPGEquations::operator[](const std::size_t i) const
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("CPGEquations::[]");
#endif //BT_NO_PROFILE
	double nodeValue;
	if (i >= nodeList.size())
	{
		nodeValue = NAN;
		throw std::invalid_argument("Node index out of bounds");
	}
	else
	{
		nodeValue = (*nodeList[i]).nodeValue;
	}
	
	return nodeValue;
}

std::vector<double>& CPGEquations::getXVars() {
#ifndef BT_NO_PROFILE 
    BT_PROFILE("CPGEquations::getXVars");
#endif //BT_NO_PROFILE
	XVars.clear();
	
	for (int i = 0; i != nodeList.size(); i++){
		XVars.push_back(nodeList[i]->phiValue);
		XVars.push_back(nodeList[i]->rValue);
		XVars.push_back(nodeList[i]->rDotValue);
	}
	
	return XVars;
}

std::vector<double>& CPGEquations::getDXVars() {
	DXVars.clear();
	
	for (int i = 0; i != nodeList.size(); i++){
		DXVars.push_back(nodeList[i]->phiDotValue);
		DXVars.push_back(nodeList[i]->rDotValue);
		DXVars.push_back(nodeList[i]->rDoubleDotValue);
	}
	
	return DXVars;
}

void CPGEquations::updateNodes(std::vector<double>& descCom)
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("CPGEquations::updateNodes");
#endif //BT_NO_PROFILE
	for(int i = 0; i != nodeList.size(); i++){
		nodeList[i]->updateDTs(descCom[i]);
	}
}

void CPGEquations::updateNodeData(std::vector<double> newXVals)
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("CPGEquations::updateNodeData");
#endif //BT_NO_PROFILE    
	assert(newXVals.size()==3*nodeList.size());
	
	for(int i = 0; i!=nodeList.size(); i++){
		nodeList[i]->updateNodeValues(newXVals[3*i], newXVals[3*i+1], newXVals[3*i+2]);
	}
}

/**
 * Function object for interfacing with ODE Int
 */
class integrate_function {
	public:
	
	integrate_function(CPGEquations* pCPGs, std::vector<double> newComs) :
	theseCPGs(pCPGs),
	descCom(newComs)
	{
		
	}
	
	void operator()  (const cpgVars_type &x ,
					cpgVars_type &dxdt ,
					double t )
	{
#ifndef BT_NO_PROFILE 
        BT_PROFILE("CPGEquations::integrate_function");
#endif //BT_NO_PROFILE
		theseCPGs->updateNodeData(x);
		theseCPGs->updateNodes(descCom);
	
		/**
		 * Read information from nodes into variables that work for ODEInt
		 */
		std::vector<double> dXVars = theseCPGs->getDXVars();
		/**
		 * Values are pre-computed by nodes, so we just have to transfer
		 * them
		 */
		for(std::size_t i = 0; i != x.size(); i++){
			dxdt[i] = dXVars[i];
		}
		//std::cout<<"operator call"<<std::endl;
		
		theseCPGs->countStep();
		
	}
	
	private:
	CPGEquations* theseCPGs;
	std::vector<double> descCom;
};

/**
 * ODE_Int Output function, can do nothing, but needs to exist
 */
class output_function {
	public: 
	
	output_function(CPGEquations* pCPGs) :
	theseCPGs(pCPGs)
	{
	}
	
	void operator() ( 	const cpgVars_type &x ,
						const double t )
	{
		/**
		 * Push integrated vars back to nodes
		 */
		theseCPGs->updateNodeData(x);
		#if (0) // Suppress output
		std::cout << t << '\t' << (*theseCPGs)[0]  << '\t' << (*theseCPGs)[1]  << '\t' << (*theseCPGs)[2]  << std::endl;
		#endif
	}
	
	private:
	CPGEquations* theseCPGs;
};

void CPGEquations::update(std::vector<double>& descCom, double dt)
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("CPGEquations::update");
#endif //BT_NO_PROFILE
	if (dt <= 0.1){ //TODO: specify default step size as a parameter during construction
		stepSize = dt;
	}
	else{
		stepSize = 0.1;
	}
	
	numSteps = 0;
	
	/**
	 * Read information from nodes into variables that work for ODEInt
	 */
	std::vector<double>& xVars = getXVars(); 
	
	/**
	 * Run ODEInt. This will change the data in xVars
	 */
	integrate(integrate_function(this, descCom), xVars, 0.0, dt, stepSize, output_function(this));
	
    if (numSteps > m_maxSteps)
    {
        std::cout << "Ending trial due to inefficient equations " << numSteps << std::endl;
        throw std::runtime_error("Inefficient CPG Parameters");
    }
    
	 #if (0)
	 std::cout << dt << '\t' << nodeList[0]->nodeValue <<
	  '\t' << nodeList[1]->nodeValue <<
	   '\t' << nodeList[2]->nodeValue << std::endl;
	 #endif
	   
}

std::string CPGEquations::toString(const std::string& prefix) const
{
	std::string p = "  ";
	std::ostringstream os;
	os << prefix << "CPGEquations(" << std::endl;

	os << prefix << p << "Nodes:" << std::endl;
	for(int i = 0; i < nodeList.size(); i++) {
		os << prefix << p << p << *(nodeList[i]) << std::endl;
	}

	os << prefix << ")" << std::endl;
	return os.str();
}
