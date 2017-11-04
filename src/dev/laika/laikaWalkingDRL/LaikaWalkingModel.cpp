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

/**
 * @file LaikaWalkingModel.cpp
 * @brief Contains the implementation of class LaikaWalkingModel
 * @author Andrew P. Sabelhaus
 * $Id$
 */

// This module
#include "LaikaWalkingModel.h"
// This library
#include "core/tgCast.h"
//#include "core/tgBasicActuator.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRigidAutoCompound.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgUtil.h"
// The Bullet Physics library
#include "btBulletDynamicsCommon.h"
// The C++ Standard Library
#include <iostream>
#include <stdexcept>
#include <math.h>

// The two constructors. Should just call the parent.
LaikaWalkingModel::LaikaWalkingModel(const std::string& structurePath) :
    TensegrityModel(structurePath) 
{
}

LaikaWalkingModel::LaikaWalkingModel(const std::string& structurePath, bool debugging):
  TensegrityModel(structurePath, debugging)
{
}


/*
void LaikaWalkingModel::mapMuscles(LaikaWalkingModel::MuscleMap& muscleMap,
            tgModel& model, size_t segmentCount)
{
    // create names for muscles (for getMuscles function)
    
    // vertical muscles
    muscleMap["vertical a"] = model.find<tgSpringCableActuator>("vertical muscle a");
    muscleMap["vertical b"] = model.find<tgSpringCableActuator>("vertical muscle b");
    muscleMap["vertical c"] = model.find<tgSpringCableActuator>("vertical muscle c");
    muscleMap["vertical d"] = model.find<tgSpringCableActuator>("vertical muscle d");
        
    // saddle muscles
    for (size_t i = 1; i < segmentCount ; i++)
    {
        muscleMap[tgString("saddle", i-1)] = model.find<tgSpringCableActuator>(tgString("saddle muscle seg", i-1));
            
    }
}
*/


/***************************************
 * The primary functions., called from other classes.
 **************************************/


void LaikaWalkingModel::setup(tgWorld& world)
{
    // Call the parent's function.
  std::cout << "Setting up the LaikaWalkingModel using TensegrityModel's YAML parser..." << std::endl;
  TensegrityModel::setup(world);

    // We could now use tgCast::filter or similar to pull out the models (e.g. muscles)
    // that we want to control.    
    //allMuscles = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());
    //mapMuscles(muscleMap, *this, m_segments);

    // Let's see what type of objects are inside the spine.
    //std::vector<tgModel*> all_children = getDescendants();
    // Pick out the tgBaseRigid objects
    //std::vector<tgBaseRigid*> all_tgBaseRigid = tgCast::filter<tgModel, tgBaseRigid>(all_children);

    // Print out the tgBaseRigids
    // std::cout << "Spine tgBaseRigids: " << std::endl;
    // for (size_t i = 0; i < all_tgBaseRigid.size(); i++)
    //   {
    // 	std::cout << "object number " << i << ": " << std::endl;
    // 	std::cout << "mass: " << all_tgBaseRigid[i]->mass() << std::endl;
    // 	std::cout << all_tgBaseRigid[i]->toString() << std::endl;
    //   }
    
    //trace(structureInfo, *this);

    // Actually setup the children
    //notifySetup();
    //tgModel::setup(world);
}


/*
void LaikaWalkingModel::step(double dt)
{
    if (dt < 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        // Step any children
        tgModel::step(dt);
    }
}
*/

/*
const std::vector<tgSpringCableActuator*>&
LaikaWalkingModel::getMuscles (const std::string& key) const
{
    const MuscleMap::const_iterator it = muscleMap.find(key);
    if (it == muscleMap.end())
    {
        throw std::invalid_argument("Key '" + key + "' not found in muscle map");
    }
    else
    {
        return it->second;
    }
}
*/

/*
const std::vector<tgSpringCableActuator*>& LaikaWalkingModel::getAllMuscles() const
{
    return allMuscles;
}
*/

