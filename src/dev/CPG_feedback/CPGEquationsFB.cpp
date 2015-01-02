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

// The Bullet Physics Library
#include "LinearMath/btQuickprof.h"

// The C++ Standard Library
#include <assert.h>
#include <stdexcept>
#include <iterator> 

using namespace boost::numeric::odeint;

typedef std::vector<double > cpgVars_type;

CPGEquationsFB::CPGEquationsFB(int maxSteps) :
CPGEquations(maxSteps)
 {}
CPGEquationsFB::CPGEquationsFB(std::vector<CPGNode*>& newNodeList, int maxSteps) :
CPGEquations(newNodeList, maxSteps)
{
}

CPGEquationsFB::~CPGEquationsFB()
{
  //CPGEquations
}

int CPGEquationsFB::addNode(std::vector<double>& newParams) 
{
	int index = nodeList.size();
	CPGNodeFB* newNode = new CPGNodeFB(index, newParams);
	nodeList.push_back(newNode);
	
	return index;
}

std::vector<double>& CPGEquationsFB::getXVars() {
#ifndef BT_NO_PROFILE 
    BT_PROFILE("CPGEquationsFB:getXVars");
#endif //BT_NO_PROFILE
    XVars.clear();
	
	for (int i = 0; i != nodeList.size(); i++){
		CPGNodeFB* currentNode = tgCast::cast<CPGNode, CPGNodeFB>(nodeList[i]);
        assert(currentNode);
		XVars.push_back(currentNode->phiValue);
		XVars.push_back(currentNode->rValue);
		XVars.push_back(currentNode->omega);
	}
	
	return XVars;
}

std::vector<double>& CPGEquationsFB::getDXVars() {
#ifndef BT_NO_PROFILE 
    BT_PROFILE("CPGEquationsFB:getDXVars");
#endif //BT_NO_PROFILE
	DXVars.clear();
	
	for (int i = 0; i != nodeList.size(); i++){
		CPGNodeFB* currentNode = tgCast::cast<CPGNode, CPGNodeFB>(nodeList[i]);
		DXVars.push_back(currentNode->phiDotValue);
		DXVars.push_back(currentNode->rDotValue);
		DXVars.push_back(currentNode->omegaDot);
	}
	
	return DXVars;
}

void CPGEquationsFB::updateNodes(std::vector<double>& descCom)
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("CPGEquationsFB:updateNodes");
#endif //BT_NO_PROFILE
	std::vector<double>::iterator comIt = descCom.begin();
	
	assert(descCom.size() == nodeList.size() * 3);
	
	for(int i = 0; i != nodeList.size(); i++){
		CPGNodeFB* currentNode = tgCast::cast<CPGNode, CPGNodeFB>(nodeList[i]);
		std::vector<double> comGroup(comIt, comIt + 3);
		currentNode->updateDTs(comGroup);
		
		comIt += 3;
	}
}

void CPGEquationsFB::updateNodeData(std::vector<double> newXVals)
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("CPGEquationsFB::updateNodeData");
#endif //BT_NO_PROFILE 
	assert(newXVals.size()==3*nodeList.size());
	
	for(int i = 0; i!=nodeList.size(); i++){
		CPGNodeFB* currentNode = tgCast::cast<CPGNode, CPGNodeFB>(nodeList[i]);
		currentNode->updateNodeValues(newXVals[3*i], newXVals[3*i+1], newXVals[3*i+2]);
	}
}
