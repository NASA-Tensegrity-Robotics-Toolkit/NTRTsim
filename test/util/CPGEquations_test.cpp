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
* @file CPGEquations_test.cpp
* @brief Contains a test of the numerical integration required for
* CPGEquations
* $Id$
*/

// This application
#include "util/CPGEquations.h"
#include "util/CPGNode.h"
// The Bullet Physics Library
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btMatrix3x3.h"
// The C++ Standard Library
#include <iostream>
#include <fstream>
// Google Test
#include "gtest/gtest.h"


using namespace std;

namespace {

	// The fixture for testing class FileHelpers.
	class CPGEquationsTest : public ::testing::Test {
		protected:
			// You can remove any or all of the following functions if its body
			// is empty.
			
			CPGEquationsTest() {
					
			}
			
			virtual ~CPGEquationsTest() {
				// You can do clean-up work that doesn't throw exceptions here.
			}
			
			// If the constructor and destructor are not enough for setting up
			// and cleaning up each test, you can define the following methods:
			virtual void SetUp() {
				// Code here will be called immediately after the constructor (right
				// before each test).
			}
			
			virtual void TearDown() {
				// Code here will be called immediately after each test (right
				// before the destructor).
			}
			
			// Objects declared here can be used by all tests in the test case.
            CPGEquations* getCPGSystem(int numNodes)
            {
                CPGEquations* m_pCPGSystem = new CPGEquations(5000);
                
                std::vector<double> params (7);
                params[0] = 1.0; // Frequency Offset
                params[1] = 0.0; // Frequency Scale
                params[2] = 1.0; // Radius Offset
                params[3] = 0.0; // Radius Scale
                params[4] = 20.0; // rConst (a constant)
                params[5] = 0.0; // dMin for descending commands
                params[6] = 5.0; // dMax for descending commands
                
                for (int i = 0; i < numNodes; i++)
                {
                    m_pCPGSystem->addNode(params);
                }
                
                /// This is a very manual means of setting up a CPG
                /// For an automated version, see tgCPGActuatorControl::setConnectivity
                
                std::vector<int> connectivityList;
                std::vector<double> weights;
                std::vector<double> phases;
                
                // Connectivity for node 0
                connectivityList.push_back(1);
                connectivityList.push_back(2);
                
                weights.push_back(1.0);
                phases.push_back(M_PI / 2.0);
                
                weights.push_back(1.0);
                phases.push_back(0.0);
                
                m_pCPGSystem->defineConnections(0, connectivityList, weights, phases); 
                connectivityList.clear();
                weights.clear();
                phases.clear();
                
                // Connectivity for node 1
                connectivityList.push_back(0);
                connectivityList.push_back(2);
                
                weights.push_back(1.0);
                phases.push_back(M_PI / 2.0);
                
                weights.push_back(1.0);
                phases.push_back(M_PI / 2.0);
                
                m_pCPGSystem->defineConnections(1, connectivityList, weights, phases); 
                connectivityList.clear();
                weights.clear();
                phases.clear();
                
                // Connectivity for node 2
                connectivityList.push_back(0);
                connectivityList.push_back(1);    
                
                weights.push_back(1.0);
                phases.push_back(0.0);
                
                weights.push_back(1.0);
                phases.push_back(M_PI / 2.0);
                
                m_pCPGSystem->defineConnections(2, connectivityList, weights, phases); 
                
                return m_pCPGSystem;
            }
	};

	TEST_F(CPGEquationsTest, testIntegration) {
            
            int numNodes = 3;
            
            // Get two identical systems to compare integration techniques
            CPGEquations* m_pCPGSystem = getCPGSystem(numNodes);
            CPGEquations* m_pCPGSystem2 = getCPGSystem(numNodes);
            
            double descendingCommand = 0.0;
            std::vector<double> desComs (numNodes, descendingCommand);
            
            // Straight integration, managed by ODE Int
            double m_updateTime = 20.0;
            m_pCPGSystem->update(desComs, m_updateTime);
            
            EXPECT_NEAR(0.0, (*m_pCPGSystem)[0], 1.0 * pow(10, -7));
            EXPECT_NEAR(-1.0, (*m_pCPGSystem)[1], 1.0 * pow(10, -7));
            EXPECT_NEAR(0.0, (*m_pCPGSystem)[2], 1.0 * pow(10, -7));
            
            int numSteps = 100;
            for (int i = 0; i < numSteps; i++)
            {
                m_pCPGSystem2->update(desComs, (m_updateTime / (double) numSteps));
            }
            
            EXPECT_NEAR((*m_pCPGSystem)[0], (*m_pCPGSystem2)[0], 1.0 * pow(10, -7));
            EXPECT_NEAR((*m_pCPGSystem)[1], (*m_pCPGSystem2)[1], 1.0 * pow(10, -7));
            EXPECT_NEAR((*m_pCPGSystem)[2], (*m_pCPGSystem2)[2], 1.0 * pow(10, -7));
            
            delete m_pCPGSystem;
            delete m_pCPGSystem2;
	}

} // namespace

int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
