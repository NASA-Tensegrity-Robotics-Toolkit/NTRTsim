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

#ifndef TETRA_SPINE_STATIC_MODEL_H
#define TETRA_SPINE_STATIC_MODEL_H

/**
 * @file TetraSpineStaticModel.h
 * @brief Tetraspine, configured for learning in the NTRT simulator
 * @author Brian Mirletz
 * @date September 2014
 * @version 1.0.0
 * $Id$
 */

// This library
#include "examples/learningSpines/BaseSpineModelLearning.h"

// Forward Declarations
class tgWorld;


/**
 * Basically the same structure as NestedStructureTestModel
 * just with different parameters and learning capabilities.
 */
class TetraSpineStaticModel: public BaseSpineModelLearning
{
public: 

    
    TetraSpineStaticModel(size_t segments);

    virtual ~TetraSpineStaticModel();
    
    virtual void setup(tgWorld& world);
    
    virtual void teardown();
    
    virtual void step(const double dt);

};

#endif
