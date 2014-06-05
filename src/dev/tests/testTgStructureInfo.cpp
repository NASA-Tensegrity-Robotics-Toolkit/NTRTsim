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

#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgBuildSpec.h"

int main(int argc, char** argv)
{
    
    tgStructure structure;
    
    structure.addNode(0,0,0, "origin");
    structure.addNode(1,2,3);
    structure.addNode(5,4,3);
    
    structure.addPair(0, 1, "rod 0-1");
    structure.addPair(1, 2, "rod 1-2");
    structure.addPair(2, 0, "rod 2-0");
    
    

    tgBuildSpec buildSpec;
    buildSpec.addBuilder("rod", new tgRodBuilder())
    
}
