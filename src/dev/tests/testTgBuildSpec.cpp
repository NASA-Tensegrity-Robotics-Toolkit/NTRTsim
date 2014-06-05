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

#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgRodInfo.h"

int main(int argc, char** argv)
{
    
    tgBuildSpec spec;
    
    const double density = 0.9;
    const double radius  = 0.3;
    const tgRod::Config rodConfig(radius, density);
    
    spec.addBuilder("rod", new tgRodInfo(rodConfig));  // @todo: should probably have tgRigidBuilder be abstract and use something like tgRodBuilder
    // spec.addBuilder("muscle", new tgConnectorBuilder());  // this is abstract -- need to use tgLinearStringBuilder or something
    
    
}
