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
 * @author Drew Sabelhaus
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

// Declare the configuration struct(s) here. This allows us to re-use the
// struct type later, and also pass it around in functions.

// A structure containing all the parameters of a spine vertebra (rigid body only.)
// This assumes that a vertebra is four rods.
struct ConfigVertebra {
    std::string vertebra_name; // a name for this type of vertebra. This is used for naming the rods when calling addRodPairs.
    double mass;  // mass of this rigid body, kg
    double radius;  // radius of the cylinders that make up this body (length)
    double leg_length;  // the length of one of the cylinders, a "leg" of the tetrahedron (length)
    double height;  // total height of one vertebra, from the bottommost node to topmost (length)
    double friction;  // A constant passed down to the underlying bullet physics solver (unitless)
    double rollFriction;  // A constant passed down to the underlying bullet physics solver (unitless)
    double restitution;  // A constant passed down to the underlying bullet physics solver (unitless)
};

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

    /**
     * What is this? Drew 2016-03-08
     */
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
     * Generate the five nodes for a vertebra tetrahedron, inside the 
     * tgStructure that's passed in, with the geometry from the 
     * ConfigVertebra that's also passed in.
     * @param[in] vertebra: the structure to build the nodes into
     * @param[in] conf_vertebra: the ConfigVertebra struct with all
     *      the information about lengths and height
     */
    static void addNodes(tgStructure& vertebra, ConfigVertebra& conf_vertebra);

    /**
     * Output debugging information for this model and structure.
     */
    static void trace(const tgStructureInfo& structureInfo, tgModel& model);
    // A version that includes the tgStructure:
    static void trace(const tgStructure& structure,
		      const tgStructureInfo& structureInfo, tgModel& model);

    /**
     * Add a tgPair for each rod in the vertebra. This maps two nodes into
     * a rod (of the vertebra_name of the ConfigVertebra).
     */
    static void addRodPairs(tgStructure& vertebra, ConfigVertebra& conf_vertebra);

    /**
     * Convert between mass and density. This is a hack! See issue 187.
     * Ideally, we'd have a constructor for tgRod or tgRigidBody that 
     * takes mass directly as an option (e.g. one construction for mass,
     * another one for density.)
     * Passing the address here guarantees that we only have one object.
     */
    static double getDensity(ConfigVertebra& conf_vertebra);

    /**
     * Calculate the edge length of a vertebra given its configuration.
     * Here, edge length means the horizontal dimension of the spine.
     * It's calculated from the length of a leg of the vertebra and total
     * vertebra height. This is done to be consistent with the anaytical
     * dynamics in MATLAB for this structure.
     */
    static double getEdgeLength(ConfigVertebra& conf_vertebra);

    /**
     * Generate the full spine structure by copying and translating one vertebra
     * into segmentCount number of vertebrae, and build that into spine.
     * DEPRECATED AS OF 2016-03-11
     */
    static void addSegments(tgStructure& spine, const tgStructure& vertebra,
			    double edge, size_t segmentCount);

    /**
     * Add one additional vertebra to the spine.
     * @param[in] spine: the tgStructure of the whole spine
     * @param[in] conf_vertebra: the config of the type of vertebra to add.
     * @param[in] vertebra_number: the number of the vertebra to add
     * TO-DO: make vertebra_number a private integer.
     */
    static void addVertebra(tgStructure& spine, ConfigVertebra& conf_vertebra,
			    size_t vertebra_number);

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
