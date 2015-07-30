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

#ifndef TG_CPG_CABLE_CONTROL_H
#define TG_CPG_CABLE_CONTROL_H

#include "examples/learningSpines/tgCPGActuatorControl.h"
#include "core/tgSpringCableActuator.h"
#include "controllers/tgPIDController.h"
// The Boost library
#include "boost/multi_array.hpp"

class tgCPGCableControl : public tgCPGActuatorControl
{
public:
 
    tgCPGCableControl(const tgPIDController::Config pid_config, const double controlStep = 1.0/10000.0);
    
    virtual ~tgCPGCableControl();
    
    virtual void onSetup(tgSpringCableActuator& subject);
    
    virtual void onStep(tgSpringCableActuator& subject, double dt);
    
    /**
     * Account for the larger number of parameters the nodes have
     * with a feedback CPGSystem
    */
    void assignNodeNumberFB (CPGEquationsFB& CPGSys, array_2D nodeParams);
    
protected:
    const tgPIDController::Config m_config;

   tgPIDController* m_PID; 
   
   bool usePID;

};


#endif
