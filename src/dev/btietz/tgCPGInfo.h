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

#ifndef TG_CPG_INFO_H
#define TG_CPG_INFO_H

#include "boost/multi_array.hpp"

class CPGEquations;
class tgConnectorInfo;
class tgBaseCPGNode;

typedef boost::multi_array<double, 2> array_2D;
typedef boost::multi_array<double, 4> array_4D;

class tgCPGInfo
{
public:

     
    const int getNodeNumber() const
    {
        return m_nodeNum;
    }   
    
    /**
     * Pointer to the CPG system. Owned by the higher level structure
     */
    const CPGEquations* getCPGSys() const
    {
        return p_CPGSys;
    }
        
    /**
     * Give the CPG information to the tgModel
     * @todo see if this can be ready for construction of the tgModel
     * the first time. Would require editing tgStructureInfo     
     */
    void addControlInfo (tgBaseCPGNode* string);
    
protected:
    /**
     * Node parameters are required right 
     */
    tgCPGInfo(CPGEquations* CPGSys, array_2D nodeParams);
    
    /**
     * Iterate through all other tgLinearStringCPGInfos, and determine
     * CPG network by rigid body connectivity
     * Can call these any time, but they'll only have the intended effect
     * after all of the strings have been constructed.
     */
    void setConnectivity (  tgConnectorInfo* thisString,
                            std::vector<tgConnectorInfo*> allStrings,
                            array_4D edgeParams);
    
private:

    int m_nodeNum;
    
    //We don't own this;
    CPGEquations* p_CPGSys;

};


#endif
