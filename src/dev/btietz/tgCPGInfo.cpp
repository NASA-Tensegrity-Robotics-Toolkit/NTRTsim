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

#include "tgCPGInfo.h"

#include "tgcreator/tgConnectorInfo.h"
#include "tgBaseCPGNode.h"

#include "tgcreator/tgRigidInfo.h"

#include "util/CPGEquations.h"
#include "core/ImpedanceControl.h"

tgCPGInfo::tgCPGInfo (CPGEquations* CPGSys, array_2D nodeParams)
{
    // Ensure that this hasn't already been assigned
    assert(m_nodeNum == -1);
    // Not a null Pointer
    assert(CPGSys);
    
    p_CPGSys = CPGSys;

    std::vector<double> params (7);
    params[0] = nodeParams[0][0]; // Frequency Offset
    params[1] = nodeParams[0][1]; // Frequency Scale
    params[2] = nodeParams[0][2]; // Radius Offset
    params[3] = nodeParams[0][3]; // Radius Scale
    params[4] = 20.0; // rConst (a constant)
    params[5] = 0.0; // dMin for descending commands
    params[6] = 5.0; // dMax for descending commands
            
    m_nodeNum = p_CPGSys->addNode(params);
} 
 
void tgCPGInfo::setConnectivity (tgConnectorInfo* thisString,
                        std::vector<tgConnectorInfo*> allStrings,
                                                array_4D edgeParams) 
{
    assert(m_nodeNum >= 0);
    
    int muscleSize = edgeParams.shape()[1];
    
    tgRigidInfo* fromGroup = thisString->getFromRigidInfo();
    tgRigidInfo* toGroup = thisString->getToRigidInfo();
    
    std::vector<int> connectivityList;
    std::vector<double> weights;
    std::vector<double> phases;
    
    /** @todo can we check to make sure the strings are on the same
    * CPG system like we used to
    */
    
    // Assuming all coupling is two way, there ought to be a way
    // to search faster than O((2N)^2) since every other
    // string has to call this. Ideas are welcome
    for (int i = 0; i < allStrings.size(); i++)
    {
        if (thisString != allStrings[i])
        {   
            tgRigidInfo* theirFromGroup = allStrings[i]->getFromRigidInfo()
                                            ->getRigidInfoGroup();
            tgRigidInfo* theirToGroup = allStrings[i]->getToRigidInfo()
                                            ->getRigidInfoGroup();
            
            // "All to all" connectivity for shared rigid bodies
            if(fromGroup == theirFromGroup || 
                toGroup == theirToGroup || 
                fromGroup == theirToGroup ||
                toGroup == theirFromGroup)
            {
                int theirMuscle = allStrings[i]->getNodeNumber();
                // Integer division: -1 is behind, 0 is same row 1 is ahead
                int rp = ((m_nodeNum - theirMuscle) / muscleSize) + 1;
                int j = m_nodeNum % muscleSize;
                int k = theirMuscle % muscleSize;
                connectivityList.push_back(theirMuscle);
                // Upper triangular matrix, lower number always goes first
                if(j > k){
                    weights.push_back(edgeParams[rp][k][j][0]);
                    phases.push_back(edgeParams[rp][k][j][1]); 
                }
                else
                {
                    weights.push_back(edgeParams[rp][j][k][0]);
                    phases.push_back(edgeParams[rp][j][k][1]); 
                }
            }
        }
    }
    
    p_CPGSys->defineConnections(m_nodeNum, connectivityList, weights, phases);
}

void tgCPGInfo::addControlInfo (tgBaseCPGNode* string)
{
    ImpedanceControl* p_ipc = new ImpedanceControl(0.0, // Tension
                                                    10000.0, // Position
                                                    5000.0); // Velocity
    
    string->setupControl(m_nodeNum, p_CPGSys, p_ipc);
    
    // string owns this pointer
    p_ipc = NULL;
}
