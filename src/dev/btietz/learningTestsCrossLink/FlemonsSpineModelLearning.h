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

#ifndef FLEMONS_SPINE_MODEL_LEARNING_H
#define FLEMONS_SPINE_MODEL_LEARNING_H

#include "dev/btietz/BaseSpineModelLearning.h"
#include "LinearMath/btVector3.h"
#include <map>
#include <set>

class tgWorld;
class tgStructureInfo;
class tgLinearString;

class FlemonsSpineModelLearning : public BaseSpineModelLearning
{
public: 

    FlemonsSpineModelLearning(int segments);

    virtual ~FlemonsSpineModelLearning();
    
    virtual void setup(tgWorld& world);
    
    virtual void teardown();
        
    virtual void step(double dt);
    
   
private:
    std::vector<tgLinearString*> reflexMuscles;
};

#endif // FLEMONS_SPINE_MODEL_H
