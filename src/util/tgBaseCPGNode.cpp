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
 * @file tgBaseCPGNode.cpp
 * @brief Implementation of class tgBaseCPGNode
 * @date April 2014
 * @author Brian Tietz
 * $Id$
 */

// This module
#include "tgBaseCPGNode.h"
// This library
#include "controllers/tgImpedanceController.h"
#include "CPGEquations.h"
// The C++ Standard Library
#include <iostream>
#include <stdexcept>

using namespace std;

tgBaseCPGNode::tgBaseCPGNode() :
  m_pMotorControl(NULL),
  m_pCPGSystem(NULL),
  m_nodeNumber(-1)
{
}

tgBaseCPGNode::~tgBaseCPGNode()
{
    delete m_pMotorControl;
    // Model deletes the CPG system
    // Should have already torn down.
}

void tgBaseCPGNode::setupControl(tgImpedanceController& ipc)
{
    if (m_nodeNumber == -1)
    {
        throw std::runtime_error("Not yet initialized");
    }
    else
    {
		m_pMotorControl = &ipc;
    }
}

double tgBaseCPGNode::getCPGValue() const
{
    double cpgValue = 0.0;
    if (m_pCPGSystem == NULL)
    {
        throw std::runtime_error("CPG not initialized!");
    }
    else
    {
        cpgValue = (*m_pCPGSystem)[m_nodeNumber];
    }
    return cpgValue;
}

tgImpedanceController& tgBaseCPGNode::motorControl() const
{
    if (m_pMotorControl == NULL)
    {
        throw std::runtime_error("Motor control has not been set.");
    }
    else
    {
        return *m_pMotorControl;
    }
}

void tgBaseCPGNode::updateTensionSetpoint(double newTension)
{
    if (newTension >= 0.0)
    {
        m_pMotorControl->setOffsetTension(newTension);
    }
    else
    {
        throw std::runtime_error("Tension setpoint is less than zero!");
    }
}

void tgBaseCPGNode::updateControlLength(double newControlLength)
{
    if (newControlLength >= 0.0)
    {
       m_controlLength = newControlLength;
    }
    else
    {
        throw std::runtime_error("Length setpoint is less than zero!");
    }
}