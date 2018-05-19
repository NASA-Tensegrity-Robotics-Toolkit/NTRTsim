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

#ifndef MODELLEARNING12BAR_H
#define MODELLEARNING12BAR_H

/**
 * @file ModelLearning12Bar.h
 * @brief Implementing learning for a 12 Bar tensegrity structure
 * @author Hannah Petersson based on code from Brian Tietz
 * @date March 2018
 * @version 1.0.0
 * $Id$
 */

#include "examples/learningSpines/BaseSpineModelLearning.h"

class tgWorld;
class tgStructureInfo;

/**
 * This class implements the octahedral complex tensegrity spine
 * based on the work of <a href="http://www.intensiondesigns.com/models.html">Tom Flemons</a>
 */
class ModelLearning12Bar : public BaseSpineModelLearning
{
public: 

    ModelLearning12Bar(int segments);

    virtual ~ModelLearning12Bar();
    
    virtual void setup(tgWorld& world);
    
    virtual void teardown();
        
    virtual void step(double dt);
    
};

#endif // MODELLEARNING12BAR_H
