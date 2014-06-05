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

#ifndef TG_BASE_STRING_CPG_H
#define TG_BASE_STRING_CPG_H

/**
 * @file tgBaseCPGNode.h
 * @brief Definition of class tgBaseCPGNode
 * @date April 2014
 * @author Brian Tietz
 * $Id$
 */

// Forward declarations
class CPGEquations;
class ImpedanceControl;

/**
 * Control class for a tgBaseString (or other mechanism.
 * using a CPG. Parent class of tgLinearStringCPG and 
 * tgRBString CPG going forward.
 * This shouldn't have children, and its child class will have tags.
 * Its child class needs to be a tgSubject
 */

class tgBaseCPGNode
{
public:
    
    virtual ~tgBaseCPGNode();                   
    
   // Top level model is responsible for stepping the CPG
   virtual double getCPGValue() const;
    
protected:

    tgBaseCPGNode();

    ImpedanceControl& motorControl() const;
	
	virtual void setupControl(ImpedanceControl& ipc); 
	
    double controlLength() const { return m_controlLength; }
    
    // We don't own this
    CPGEquations* m_pCPGSystem;
	
	int m_nodeNumber;
	
	double m_controlLength;
private:	
	// We own this
    ImpedanceControl* m_pMotorControl;

};


#endif
