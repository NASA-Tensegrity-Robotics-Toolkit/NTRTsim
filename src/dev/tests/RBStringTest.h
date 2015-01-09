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

#ifndef RB_STRING_TEST_H
#define RB_STRING_TEST_H

#include "core/tgModel.h" 

#include "core/tgSubject.h"

// SubModels
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"

#include <set>
#include <map>

class tgWorld;
class CPGEquations;
class tgNode;

class RBStringTest: public tgSubject<RBStringTest>, public tgModel
{
public: 
    
    struct Config
    {
    public:
        Config( int segments,
                const tgRod::Config& rodConf,
                const tgBasicActuator::Config& stringConf,
                double minTotalLength = 0.1); // todo: find better default
        
        int m_segments;
        tgRod::Config           m_rodConfig;
        tgBasicActuator::Config  m_stringConfig;
        double m_minTotalLength;
    };
    
    RBStringTest(tgNode* start, tgNode* end, const RBStringTest::Config& config);

    virtual ~RBStringTest()
    {}
    
    virtual void setup(tgWorld& world);
    
    // @todo: Is there a way that we can get rid of the need to override this function in subclasses of tgModel? 
    // comment_BRT: only if we can somehow make tgModel a template class,
    // we need to know what class we're notifying
    virtual void step(double dt);
    
    const int getSegments() const
    {
        return m_config.m_segments;
    }
    
private:
    std::vector<tgBasicActuator*> allMuscles;

    Config m_config;
    
    // 
    const tgNode* m_pStartNode;
    const tgNode* m_pEndNode;
};

#endif // RB_STRING_TEST_H
