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

#ifndef SIMULATOR_SRC_LIB_MODELS_SNAKE_CPGS_CPGEDGE
#define SIMULATOR_SRC_LIB_MODELS_SNAKE_CPGS_CPGEDGE

/**
 * @file CPGEdge.h
 * @brief Definition of class CPGEdge
 * @date March 2014
 * @author Brian Tietz
 * $Id$
 */

#include <math.h> 
#include <sstream>
#include <vector>

#include "CPGNode.h"

class CPGNode; //Forward declaration

/**
 * Defines the connectivity between two CPGNode objects
 */
class CPGEdge
{
	public:
	CPGEdge(CPGNode* newTarget,
			double newWeight,
			double newPhase);
	/**
	 * todo: double check cleanup and make sure this needs to delete its nodes
	 */
	~CPGEdge();
	
	/**
	 * Equation that affects the phase of the node based on the
	 * coupled node. Can be overridden to produce a different
	 * coupling equation.
	 */
	virtual void couple(CPGNode & currentNode);
	
	CPGNode* getTargetNode(){
		return targetNode;
	}
	
	std::string toString(const std::string& prefix = "") const;
	
	protected:
	
	CPGNode* targetNode;
	
	double weight;
	
	double phaseOffset;
	
};

/**
 * Overload operator<<() to handle CPGNode
 * @param[in,out] os an ostream
 * @param[in] a reference to a CPGNode
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 */
inline std::ostream&
operator<<(std::ostream& os, const CPGEdge& obj)
{
    os << obj.toString() << std::endl;
	return os;
}

#endif // SIMULATOR_SRC_LIB_MODELS_SNAKE_CPGS_CPGEDGE
