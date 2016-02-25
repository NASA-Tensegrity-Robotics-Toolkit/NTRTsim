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

#ifndef VERTICAL_SPINE_MODEL_H
#define VERTICAL_SPINE_MODEL_H

/**
 * @file VerticalSpineModel.h
 * @brief Contains the definition of class VerticalSpineModel
 * @author Brian Tietz, Drew Sabelhaus, Michael Fanton
 * $Id$
 */

// This library
#include "core/tgModel.h" 
#include "core/tgSubject.h"
// The C++ Standard Library
#include <map>
#include <set>
#include <string>
#include <vector>

#include "sensors/tgDataObserver.h"

// Forward declarations
class tgSpringCableActuator;
class tgStructure;
class tgStructureInfo;

class VerticalSpineModel: public tgSubject<VerticalSpineModel>, public tgModel
{
public: 
	
	/**
	 * Used within this function to map segments to string keys
	 */
    typedef std::map<std::string, std::vector<tgSpringCableActuator*> > MuscleMap;
	
	/**
	 * The only constructor. The model details are instantiated once
	 * setup is called typically when the model is added to a simulation.
	 * @param[in] segments, a positive integer dictating how many
	 * rigid segments will be constructed. 
	 */
    VerticalSpineModel(size_t segments);
	
	/**
	 * Nothing to do. Most functions already handled by tgModel::teardown
	 */
    virtual ~VerticalSpineModel()
    {}
    
    /**
     * Create the model. Place the rods and strings into the world
     * that is passed into the simulation. This is triggered
     * automatically when the model is added to the simulation, when
     * tgModel::setup(world) is called (if this model is a child),
     * and when reset is called. Also notifies controllers of setup.
     * @param[in] world - the world we're building into
     */
    virtual void setup(tgWorld& world);
	
	/**
     * Step the model, its children. Notifies controllers of step.
     * @param[in] dt, the timestep. Must be positive.
     */
    virtual void step(const double dt);
	
	/**
	 * Get a group of muscles according to the provided key. Groups are
	 * populated during setup.
	 * @param[in] key - a std:string* used as a key to a std::map from
	 * a string to a vector of muscles
	 * @return a std::vector of pointers to the muscles found by the key
	 */
    const std::vector<tgSpringCableActuator*>& getMuscles (const std::string& key) const;
     
    const std::vector<tgSpringCableActuator*>& getAllMuscles() const;
    
    /**
     * Return a std::size_t indicating the number of segments in the 
     * tetraSpine.
     * @return a std::size_t with the value of m_segments
     */
    size_t getSegments() const
    {
        return m_segments;
    }
    
private:
	
    /**
     * A series of helper functions, used to create the model.
     */

    /**
     * Generate the five nodes for a tetrahedron, inside the tgStructure
     * that's passed in.
     * @param[in] tetra: the structure to build the nodes into
     * @param[in] edge: edge length for this one tetrahedron
     * @param[in] height: height length for this one tetrahedron
     */
    static void addNodes(tgStructure& tetra, double edge, double height);

    /**
     * Output debugging information for this model and structure.
     */
    static void trace(const tgStructureInfo& structureInfo, tgModel& model);

    /**
     * Pair together the nodes for the first type of rod
     */
    static void addPairs(tgStructure& tetra);

    /**
     * Pair together the nods for the second type of rod
     */
    static void addPairsB(tgStructure& tetra);

    /**
     * Generate the full spine structure by copying and translating one tetra
     * into segmentCount number of tetras, and build that into spine.
     */
    static void addSegments(tgStructure& spine, const tgStructure& tetra, double
			    edge, size_t segmentCount);

    /**
     * Add the muscles that connect between the segments of the spine.
     */
    static void addMuscles(tgStructure& spine);

    /**
     * Create the names for the muscles, used in the getMuscles function.
     */
    static void mapMuscles(VerticalSpineModel::MuscleMap& muscleMap, tgModel&
			   model, size_t segmentCount);

	/**
	 * A std::vector containing all of the tgStringCableActuators amongst
	 * the children of this model. Populated during setup
	 */
    std::vector<tgSpringCableActuator*> allMuscles;
	
	/**
	 * A typdef of std::map from std::string to tgLinearMuscle*. Contains
	 * mapped muscles, populated during setup.
	 */
    MuscleMap muscleMap;
    
    /**
     * The number of segments in the spine
     */
    const size_t m_segments;
};

#endif
