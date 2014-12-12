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

#ifndef TETRA_SPINE_CPG_CONTROL_H
#define TETRA_SPINE_CPG_CONTROL_H

/**
 * @file TetraSpineCPGControl.h
 * @brief Controller for TetraSpineLearningModel
 * @author Brian Tietz
 * @date May 2014
 * @version 1.0.0
 * $Id$
 */

#include "examples/learningSpines/BaseSpineCPGControl.h"

/**
 * Inherits from BaseSpineCPGControl, and overrides setupCPGs so
 * different muscle groups can have different tgImpedanceController parameters
 */
class TetraSpineCPGControl : public BaseSpineCPGControl
{
public:

    TetraSpineCPGControl(BaseSpineCPGControl::Config config,	
							std::string args,
							std::string resourcePath,
                            std::string ec = "edgeConfig.ini",
                            std::string nc = "nodeConfig.ini");
    
    ~TetraSpineCPGControl() {}

	virtual void setupCPGs(BaseSpineModelLearning& subject, array_2D nodeActions, array_4D edgeActions);
	
};

#endif // FLEMONS_SPINE_CPG_CONTROL_H
