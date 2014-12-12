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

#include "CPGEquationsFB.h"

#include "core/tgCast.h"

#include "boost/array.hpp"
#include "boost/numeric/odeint.hpp"

// The C++ Standard Library
#include <assert.h>
#include <stdexcept>
#include <iterator> 

using namespace boost::numeric::odeint;

typedef std::vector<double > cpgVars_type;

CPGEquationsFB::CPGEquationsFB() :
CPGEquations()
 {}
CPGEquationsFB::CPGEquationsFB(std::vector<CPGNode*> newNodeList) :
CPGEquations(newNodeList)
{
}

CPGEquationsFB::~CPGEquationsFB()
{
  //CPGEquations
}

std::vector<double> CPGEquationsFB::getXVars() {
	std::vector<double> newXVars;
	
	for (int i = 0; i != nodeList.size(); i++){
		CPGNodeFB* currentNode = tgCast::cast<CPGNode, CPGNodeFB>(nodeList[i]);
		newXVars.push_back(currentNode->phiValue);
		newXVars.push_back(currentNode->rValue);
		newXVars.push_back(currentNode->omega);
	}
	
	return newXVars;
}

std::vector<double> CPGEquationsFB::getDXVars() {
	std::vector<double> newDXVars;
	
	for (int i = 0; i != nodeList.size(); i++){
		CPGNodeFB* currentNode = tgCast::cast<CPGNode, CPGNodeFB>(nodeList[i]);
		newDXVars.push_back(currentNode->phiDotValue);
		newDXVars.push_back(currentNode->rDotValue);
		newDXVars.push_back(currentNode->omegaDot);
	}
	
	return newDXVars;
}

void CPGEquationsFB::updateNodes(std::vector<double> descCom)
{
	std::vector<double>::iterator comIt = descCom.begin();
	
	assert(descCom.size() == nodeList.size() * 3);
	
	for(int i = 0; i != nodeList.size(); i++){
		CPGNodeFB* currentNode = tgCast::cast<CPGNode, CPGNodeFB>(nodeList[i]);
		std::vector<double> comGroup(comIt, comIt + 2);
		currentNode->updateDTs(comGroup);
		
		comIt += 3;
	}
}

void CPGEquationsFB::updateNodeData(std::vector<double> newXVals)
{
	assert(newXVals.size()==3*nodeList.size());
	
	for(int i = 0; i!=nodeList.size(); i++){
		CPGNodeFB* currentNode = tgCast::cast<CPGNode, CPGNodeFB>(nodeList[i]);
		currentNode->updateNodeValues(newXVals[3*i], newXVals[3*i+1], newXVals[3*i+2]);
	}
}
