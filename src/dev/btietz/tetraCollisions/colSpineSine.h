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

#ifndef COL_SPINE_SINE_H
#define COL_SPINE_SINE_H

/**
 * @file colSpineSine.h
 * @brief Controller for TetraSpineCollisions
 * @author Brian Mirletz
 * @date November 2014
 * @version 1.0.0
 * $Id$
 */

#include "examples/learningSpines/BaseSpineCPGControl.h"
#include "core/tgObserver.h"

#include "tscDataObserver.h"

class tgSineStringControl;

/**
 * Inherits from BaseSpineCPGControl, and overrides setupCPGs so
 * different muscle groups can have different ImpedanceControl parameters
 * Modified from htSpineSine to accomidate additional segments
 */
class colSpineSine : public tgObserver<BaseSpineModelLearning>
{
public:
    colSpineSine(std::string args,
                 std::string resourcePath = "");
    
    ~colSpineSine() {}
	
	virtual void onSetup(BaseSpineModelLearning& subject);
	
	virtual void onStep(BaseSpineModelLearning& subject, double dt);
    
    virtual void onTeardown(BaseSpineModelLearning& subject);

protected:
	
	virtual void setupWaves(BaseSpineModelLearning& subject);
	
	std::vector<tgSineStringControl*> m_sineControllers;
	
	tscDataObserver m_dataObserver;
	
	double m_updateTime;
	
	double m_controlTime;
    std::vector<double> initConditions;
    std::string controlFilename;
    std::string controlFilePath;
};

#endif // HT_SPINE_SINE_H
