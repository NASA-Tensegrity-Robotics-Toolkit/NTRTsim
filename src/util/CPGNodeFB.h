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

#ifndef CPG_FEEDBACK_CPG_NODE_FB
#define CPG_FEEDBACK_CPG_NODE_FB

/**
 * @file CPGNodeFB.h
 * @brief Definition of class CPGNodeFB
 * @date December 2014
 * @author Brian Mirletz
 * $Id$
 */

#include "util/CPGNode.h"

#include <vector>
#include <sstream>

//Forward Declaration
class CPGEdge; 

class CPGNodeFB : public CPGNode
{
	friend class CPGEquationsFB;
	
	public:
	
	/**
	 * @todo consider adding vector of initial conditions for
	 * stability
	 */
	CPGNodeFB(int nodeNum, const std::vector<double> & params);
	
	virtual ~CPGNodeFB();
	
	/**
	 * Update phiDotValue and rDoubleDotValue based on Node equations and
	 * coupling equations
	 * @todo better name?
	 */
	virtual void updateDTs(const std::vector<double>& feedback);
			
	void updateNodeValues (	double newR,
							double newPhi,
							double newO);

	protected:
	
	/**
	 * Values for numerical integration
	 */
	double omega;
	double omegaDot;
	
	const double kFreq;
	const double kAmp;
	const double kPhase;
	
};
#endif // SIMULATOR_SRC_LIB_MODELS_SNAKE_CPGS_CPGNODE
