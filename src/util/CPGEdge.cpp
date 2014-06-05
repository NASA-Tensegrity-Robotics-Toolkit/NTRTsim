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
 * @file CPGEdge.cpp
 * @brief Definition of class CPGEdge
 * @date March 2014
 * @author Brian Tietz
 * $Id$
 */

#include "CPGEdge.h"
#include "CPGNode.h"

CPGEdge::CPGEdge(	CPGNode* newTarget,
					double newWeight,
					double newPhase):
targetNode(newTarget),
weight(newWeight),
phaseOffset(newPhase)
{
		
}

CPGEdge::~CPGEdge(){
	// will be deleted by CPGEquations
	targetNode = NULL;
}

void CPGEdge::couple(CPGNode & currentNode){
	currentNode.phiDotValue += weight * targetNode->rValue * sin (targetNode->phiValue - currentNode.phiValue - phaseOffset);
}

std::string CPGEdge::toString(const std::string& prefix) const
{
	std::string p = "  ";
	std::ostringstream os;
	os << prefix << "CPGEdge(";

	// TODO: add something about parameters of this edge?
	
	os << prefix << p << "Conected to:";
	os << prefix << p << p << targetNode->getNodeIndex();

	os << prefix << ")";
	return os.str();
}
