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

// The C++ Standard Library
#include <assert.h>
#include <stdexcept>

using namespace boost::numeric::odeint;

typedef std::vector<double > cpgVars_type;

CPGEquations::CPGEquations() :
stepSize(0.1) 
 {}
CPGEquations::CPGEquations(std::vector<CPGNode*>& newNodeList) :
nodeList(newNodeList),
stepSize(0.1) //TODO: specify as a parameter somewhere
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
int CPGEquations::addNode(std::vector<double> newParams) 
{
	int index = nodeList.size();
	CPGNode* newNode = new CPGNode(index, newParams);
	nodeList.push_back(newNode);
	
	return index;
}
				
void CPGEquations::connectNode(	int nodeIndex,
								std::vector<CPGEdge*> connectivityList)
{
	nodeList[nodeIndex]->addCoupling(connectivityList);
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

std::vector<double> CPGEquations::getXVars() {
	std::vector<double> newXVars;
	
	for (int i = 0; i != nodeList.size(); i++){
		newXVars.push_back(nodeList[i]->phiValue);
		newXVars.push_back(nodeList[i]->rValue);
		newXVars.push_back(nodeList[i]->rDotValue);
	}
	
	return newXVars;
}

std::vector<double> CPGEquations::getDXVars() {
	std::vector<double> newDXVars;
	
	for (int i = 0; i != nodeList.size(); i++){
		newDXVars.push_back(nodeList[i]->phiDotValue);
		newDXVars.push_back(nodeList[i]->rDotValue);
		newDXVars.push_back(nodeList[i]->rDoubleDotValue);
	}
	
	return newDXVars;
}

void CPGEquations::updateNodes(std::vector<double>& descCom)
{
	for(int i = 0; i != nodeList.size(); i++){
		nodeList[i]->updateDTs(descCom[i]);
	}
}

void CPGEquations::updateNodeData(std::vector<double> newXVals)
{
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
	if (dt <= 0.1){ //TODO: specify default step size as a parameter during construction
		stepSize = dt;
	}
	else{
		stepSize = 0.1;
	}
	
	/**
	 * Read information from nodes into variables that work for ODEInt
	 */
	std::vector<double> xVars = getXVars(); 
	
	/**
	 * Run ODEInt. This will change the data in xVars
	 */
	integrate(integrate_function(this, descCom), xVars, 0.0, dt, stepSize, output_function(this));
	 
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
