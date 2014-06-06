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

#ifndef RIB_MODEL_H
#define RIB_MODEL_H

/**
 * @file RibModel.h
 * @brief Implements a spine model with a rib cage
 * @author Brian Tietz
 * @date May 2014
 * @version 1.0.0
 * $Id$
 */

#include "examples/learningSpines/BaseSpineModelLearning.h"

class tgWorld;

/**
 * The spine model is similar in shape to FlemonsSpineModelLearning, the
 * ribs are rigidly attached ellipses.
 */
class RibModel: public BaseSpineModelLearning
{
public: 
    
    RibModel(int segments);

    virtual ~RibModel();

    virtual void setup(tgWorld& world);
    
    virtual void teardown();    
    
    virtual void step(double dt);

};

#endif // RIB_MODEL_H
