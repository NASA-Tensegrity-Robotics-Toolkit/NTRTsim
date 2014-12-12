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
 * @file CPGNodeFB.cpp
 * @brief Implementation of class CPGNodeFB
 * @date December 2014
 * @author Brian Mirletz
 * $Id$
 */

#include "CPGNodeFB.h"
#include "util/CPGEdge.h"

// The C++ Standard Library
#include <algorithm> //for_each
#include <math.h> 
#include <assert.h>

CPGNodeFB::CPGNodeFB(int nodeNum, const std::vector<double> & params):
omega(params[7]),
omegaDot(0),
kFreq(params[8]),
kAmp(params[9]),
kPhase(params[10]),
CPGNode(nodeNum, params)
{
	//Precondition
	assert(params.size() >= 11);
}

CPGNodeFB::~CPGNodeFB()
{

}
		
void CPGNodeFB::updateDTs(std::vector<double> feedback)
{
	assert(feedback.size() >= 3);
	
	phiDotValue = omega + kPhase * feedback [2];
	
	/**
	 * Iterate through every edge and affect the phase of this node
	 * accordingly.
	 * @todo ask about refactoring to use for_each
	 */
	for (int i = 0; i != couplingList.size(); i++){
		couplingList[i]->couple(*this);
	}
	
	omegaDot = kFreq * feedback[0] * sin(phiValue);
	
	rDotValue = rConst * (radiusOffset + kAmp * feedback[1] - pow(rValue, 2.0)) * rValue;
}

void CPGNodeFB::updateNodeValues (double newPhi,
								double newR,
								double newO)
								
{
	rValue = newR;
	phiValue = newPhi;
	omega = newO;
	nodeValue = rValue*cos(phiValue);
}
