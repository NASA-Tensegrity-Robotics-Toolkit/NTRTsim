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
 * @file CPGNode.cpp
 * @brief Implementation of class CPGNode
 * @date March 2014
 * @author Brian Tietz
 * $Id$
 */

#include "CPGNode.h"
//#include "CPGEdge.h"

// The C++ Standard Library
#include <algorithm> //for_each
#include <math.h> 
#include <assert.h>

CPGNode::CPGNode(int nodeNum, const std::vector<double> & params):
nodeValue(0),
phiValue(0),
phiDotValue(0),
rValue(0),
rDotValue(0),
m_nodeNumber(nodeNum),
rDoubleDotValue(0),
rConst(params[4]),
frequencyOffset(params[0]),
frequencyScale(params[1]),
radiusOffset(params[2]),
radiusScale(params[3]),
dMin(params[5]),
dMax(params[6])
{
	//Precondition
	assert(params.size() >= 7);
}

CPGNode::~CPGNode()
{
	couplingList.clear();
}

void CPGNode::addCoupling(	CPGNode* cNode,
							const double cWeight,
							const double cPhase)
{
	couplingList.push_back(cNode);
    weightList.push_back(cWeight);
    phaseList.push_back(cPhase);
    
    assert(couplingList.size() == weightList.size() && couplingList.size() == phaseList.size());
}
	
void CPGNode::updateDTs(double descCom)
{
	phiDotValue = 2 * M_PI * nodeEquation(descCom, frequencyOffset, frequencyScale);
	
	/**
	 * Iterate through every edge and affect the phase of this node
	 * accordingly.
	 * @todo ask about refactoring to use for_each
	 */
	const std::size_t n = couplingList.size();
	for (std::size_t i = 0; i != n; i++){
        const CPGNode& targetNode = *(couplingList[i]);
        phiDotValue += weightList[i] * targetNode.rValue * sin (targetNode.phiValue - phiValue - phaseList[i]);
	}
	
	rDoubleDotValue = rConst * (rConst / 4 * (nodeEquation(descCom, radiusOffset, radiusScale)
		- rValue) - rDotValue);
}

double CPGNode::nodeEquation(	double d,
								double c0,
								double c1)
{
	//@todo: Make parameters dMax, dMin
	if(d >= dMin && d <= dMax){ 
		return c1 * d + c0;
	}
	else{
		return 0;
	}
}
				
void CPGNode::updateNodeValues (double newPhi,
								double newR,
								double newRD)
								
{
	rValue = newR;
	rDotValue = newRD;
	phiValue = newPhi;
	nodeValue = rValue*cos(phiValue);
}

std::string CPGNode::toString(const std::string& prefix) const
{
	std::string p = "  ";
	std::ostringstream os;
	os << prefix << "CPGNode(" << p << m_nodeNumber << std::endl;

	// TODO: add something about parameters of this node?
	
	os << prefix << p << "Connectivity:" << std::endl;
	for(int i = 0; i < couplingList.size(); i++) {
		os << prefix << p << p << *(couplingList[i]);
	}

	os << prefix << ")";
	return os.str();
}
