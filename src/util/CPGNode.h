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

#ifndef SIMULATOR_SRC_LIB_MODELS_SNAKE_CPGS_CPGNODE
#define SIMULATOR_SRC_LIB_MODELS_SNAKE_CPGS_CPGNODE

/**
 * @file CPGNode.h
 * @brief Definition of class CPGNode
 * @date March 2014
 * @author Brian Tietz
 * $Id$
 */


#include <vector>
#include <sstream>

//Forward Declaration
class CPGEdge; 

class CPGNode
{
	friend class CPGEquations;
	friend class CPGNodeFB;
    
	public:
	
	/**
	 * @todo consider adding vector of initial conditions for
	 * stability
	 */
	CPGNode(int nodeNum, const std::vector<double> & params);
	virtual ~CPGNode();

	void addCoupling(	CPGNode* cNode,
						const double cWeight,
						const double cPhase);

	/**
	 * Update phiDotValue and rDoubleDotValue based on Node equations and
	 * coupling equations
	 * @todo better name?
	 */
	virtual void updateDTs(	double descCom);
	
	/**
	 * Compute the base node equation for R and Phi
	 */
	double nodeEquation(	double d,
							double c0,
							double c1);
				
	virtual void updateNodeValues (	double newR,
									double newRD,
									double newPhi);
	// out of date since we're using pointers, but still potentially useful
	#if (0)
	std::vector<CPGNode*> getCoupling(){
		std::vector<CPGNode*> couplingNumbers;
		return couplingNumbers;
	}
	#endif
	
	const int getNodeIndex() const
	{
		return m_nodeNumber;
	}
	
	std::string toString(const std::string& prefix = "") const;
    
	protected:
	
	/**
	 * Values for numerical integration
	 */
	double nodeValue;
	double phiValue;
	double phiDotValue;
	double rValue; //Radius
	double rDotValue; //rDot for next update
	double rDoubleDotValue; //Deriviative of RDot
	
	std::vector<CPGNode*> couplingList;
	std::vector<double> phaseList;
    std::vector<double> weightList;
    
	/**
	 * Index of this node for printing and debugging
	 */
	const int m_nodeNumber;
	
	/**
	 * Parameters for node equations:
	 */
	const double rConst;
	
	const double frequencyOffset;
	const double frequencyScale;
	
	const double radiusOffset;
	const double radiusScale;
	
	const double dMin;
	const double dMax;
	
};

/**
 * Overload operator<<() to handle CPGNode
 * @param[in,out] os an ostream
 * @param[in] a reference to a CPGNode
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 */
inline std::ostream&
operator<<(std::ostream& os, const CPGNode& obj)
{
    os << obj.toString() << std::endl;
	return os;
}

#endif // SIMULATOR_SRC_LIB_MODELS_SNAKE_CPGS_CPGNODE
