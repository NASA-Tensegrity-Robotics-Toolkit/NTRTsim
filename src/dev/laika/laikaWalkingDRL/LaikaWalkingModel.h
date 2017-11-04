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

#ifndef LAIKA_WALKING_MODEL_H
#define LAIKA_WALKING_MODEL_H

/**
 * @file LaikaWalkingModel.h
 * @brief Contains the definition of class LaikaWalkingModel
 * @author Drew Sabelhaus
 * $Id$
 */

// This library
#include "yamlbuilder/TensegrityModel.h"
// The C++ Standard Library
#include <map>
#include <set>
#include <string>
#include <vector>

// Forward declarations
//class tgSpringCableActuator;
//class tgStructure;
//class tgStructureInfo;

// This class will inherit from TensegrityModel, and we don't really
// need to do much else besides add some extra methods to get the rigid bodies
// and cables.
class LaikaWalkingModel: public TensegrityModel
{
public: 
	
    /**
     * The two constructors. Will just pass in to the parent
     */
    LaikaWalkingModel(const std::string& structurePath);
    LaikaWalkingModel(const std::string& structurePath, bool debugging);
	
    /**
     * Nothing to do. Most functions already handled by tgModel::teardown
     */
    virtual ~LaikaWalkingModel()
    {}
    
    /**
     * Create the model. Call the parent's method,
     * then add a little extra in.
     */
    virtual void setup(tgWorld& world);
	
    /**
     * Step the model, its children. Notifies controllers of step.
     * @param[in] dt, the timestep. Must be positive.
     */
    //virtual void step(const double dt);
	
    /**
     * Get a group of muscles according to the provided key. Groups are
     * populated during setup.
     * @param[in] key - a std:string* used as a key to a std::map from
     * a string to a vector of muscles
     * @return a std::vector of pointers to the muscles found by the key
     */
    //const std::vector<tgSpringCableActuator*>& getMuscles (const std::string& key) const;

    /**
     * What is this? Drew 2016-03-08
     */
    //const std::vector<tgSpringCableActuator*>& getAllMuscles() const;
    
    /**
     * Return a std::size_t indicating the number of segments in the 
     * tetraSpine.
     * @return a std::size_t with the value of m_segments
     */
    /*
      size_t getSegments() const
    {
        return m_segments;
    }
    */
    
private:
	

    /**
     * Add the muscles that connect between the segments of the spine.
     */
    //static void addMuscles(tgStructure& spine);

    /**
     * Create the names for the muscles, used in the getMuscles function.
     */
    //static void mapMuscles(VerticalSpineModel::MuscleMap& muscleMap, tgModel&
    //			   model, size_t segmentCount);

    /**
     * A std::vector containing all of the tgStringCableActuators amongst
     * the children of this model. Populated during setup
     */
    //std::vector<tgSpringCableActuator*> allMuscles;
	
    /**
     * A typdef of std::map from std::string to tgLinearMuscle*. Contains
     * mapped muscles, populated during setup.
     */
    //MuscleMap muscleMap;
    
};

#endif
