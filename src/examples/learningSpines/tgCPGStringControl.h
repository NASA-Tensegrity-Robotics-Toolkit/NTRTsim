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

#ifndef TG_CPG_STRING_CONTRL_H
#define TG_CPG_STRING_CONTRL_H

#include "util/tgBaseCPGNode.h"
#include "core/tgLinearString.h"
// The Boost library
#include "boost/multi_array.hpp"

typedef boost::multi_array<double, 2> array_2D;
typedef boost::multi_array<double, 4> array_4D;

// Forward declarations
class btRigidBody;
class CPGEquations;
class CPGEquationsFB;
class ImpedanceControl;

class tgCPGStringControl : public tgObserver<tgLinearString>,
							public tgBaseCPGNode
{
public:
 
    tgCPGStringControl(const double controlStep = 1.0/10000.0);
    
    virtual ~tgCPGStringControl();
    
    virtual void onAttach(tgLinearString& subject);
    
    virtual void onStep(tgLinearString& subject, double dt);
	
	/**
     * Can call these any time, but they'll only have the intended effect
     * after all of the strings have been constructed.
     */
    
    void assignNodeNumber (CPGEquations& CPGSys, array_2D nodeParams);
    
    /**
     * Account for the larger number of parameters the nodes have
     * with a feedback CPGSystem
     */
    void assignNodeNumberFB (CPGEquationsFB& CPGSys, array_2D nodeParams);
    
    /**
     * Iterate through all other tgLinearStringCPGInfos, and determine
     * CPG network by rigid body connectivity
     */
    void setConnectivity(const std::vector<tgCPGStringControl*>& allStrings,
             array_4D edgeParams);
    
    const int getNodeNumber() const
    {
        return m_nodeNumber;
    }   
    
    /**
     * Pointer to the CPG system. Owned by the higher level controller
     */
    const CPGEquations* getCPGSys() const
    {
        return m_pCPGSystem;
    }
    
    const double getCommandedTension() const
    {
        return m_commandedTension;
    }
    
    virtual void setupControl(ImpedanceControl& ipc);
    
    void setupControl(ImpedanceControl& ipc,
						double controlLength);

	const btRigidBody* getFromBody() const
	{
		return m_pFromBody;
	}
	
	const btRigidBody* getToBody() const
	{
		return m_pToBody;
	}
	
protected:
    /**
     * Member variable for keeping track of how long its been since the 
     * last update step
     */
    double m_controlTime;
    /**
     * How often this controller updates. Must be non-negative. Zero
     * means it updates every timestep. Units are seconds
     */
    const double m_controlStep;
    
    double m_totalTime;
    
    double m_commandedTension;
private:    
    btRigidBody* m_pFromBody;
    
    btRigidBody* m_pToBody;
};


#endif
