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
 * @file VerticalSpineModel.cpp
 * @brief Contains the implementation of class VerticalSpineModel
 * @author Andrew P. Sabelhaus
 * $Id$
 */

// This module
#include "VerticalSpineModel.h"
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

// The single constructor.
VerticalSpineModel::VerticalSpineModel(size_t segments) :
    m_segments(segments),
    tgModel() 
{
}

/**
 * See the header file for the declaration of various struct types.
 * These are used to create different vertebra nice and easily!
 * Note that we have different configurations here: a config for...
 *      - first base vertebra (unmoving)
 *      - passive vertebra (moving, the ones without the actuator mass)
 *      - active vertebra (moving, with actuator mass)
 *      - vertical cables (vertical muscles)
 *      - saddle cables (saddle muscles)
 *      - the spine as a whole (initial separation between vertebrae, etc.)
 * Note that this configuration below does NOT enforce any angle between rods!
 * So, this is NOT necessarily a symmetric tetrahedron.
 * To get one that's symmetric, set height == edge, or equivalently, leg_length == height / sqrt(2).
 */
namespace
{
  ConfigVertebra conf_base_vertebra = {
    "base", // the name of this type of vertebra
    0.0,  // mass. Note that setting a rigid body's mass to 0 makes it fixed in space.
    0.5,  // radius
    12.25,  // leg_length. Was calculated on 2016-03-08 from sqrt( (height/2)^2 + (edge/2)^2 )
    14.14,  // height. Was calculated on 2016-03-08 from edge / sqrt(2). Previously, edge = 20.
    0.99,  // friction
    0.01,  // rollFriction
    0.0,  // restitution
  };

  // On 2016-03-08, the two-segment model of ULTRA Spine (one active, one passive)
  // weighed 231g. Estimate: 2/5 passive (92.4), 3/5 active (138.6).
  ConfigVertebra conf_passive_vertebra = {
    "passive", // the name of this type of vertebra
    0.0924,  // mass
    0.5,  // radius
    12.25,  // leg_length. Was calculated on 2016-03-08 from sqrt( (height/2)^2 + (edge/2)^2 )
    14.14,  // height. Was calculated on 2016-03-08 from edge / sqrt(2). Previously, edge = 20.
    0.99,  // friction
    0.01,  // rollFriction
    0.0,  // restitution
  };

  ConfigVertebra conf_active_vertebra = {
    "active", // the name of this type of vertebra
    0.1386,  // mass
    0.5,  // radius
    12.25,  // leg_length. Was calculated on 2016-03-08 from sqrt( (height/2)^2 + (edge/2)^2 )
    14.14,  // height. Was calculated on 2016-03-08 from edge / sqrt(2). Previously, edge = 20.
    0.99,  // friction
    0.01,  // rollFriction
    0.0,  // restitution
  };

  struct ConfigSpine {
    double vertebra_separation; // Length in the vertical direction of the initial separation between adjacent vertebrae
    double base_vertical_offset; // The bottom vertebra clips below the ground a bit. Move it up by this amount.
  } conf_spine = {
    7.5,  // vertebra_separation
    2.0  // base_vertical_offset
  };
  
  const struct Config
    {
        double densityA;
        double densityB;
        double radius;
        double edge;
        double height;
        double stiffness;
        double damping;  
        double friction;
        double rollFriction;
        double restitution;
        double pretension;
        bool   hist;
        double maxTens;
        double targetVelocity;
    } c =
   {
     // On 2016-03-08, the two-segment model of ULTRA Spine (one active, one passive)
     // weighed 231g. 
     0.026,    // densityA (kg / length^3)
     0.0,    // densityB (kg / length^3)
     0.5,     // radius (length)
     20.0,      // edge (length)
     tgUtil::round(c.edge / std::sqrt(2.0)),    // height (length)
     1000.0,   // stiffness (kg / sec^2)
     10.0,    // damping (kg / sec)
     0.99,      // friction (unitless)
     0.01,     // rollFriction (unitless)
     0.0,      // restitution (?)
     2452.0,        // pretension. used to be 2452.0
     0,			// History logging (boolean)
     100000,   // maxTens
     10000,    // targetVelocity
      
  };

} // end namespace

// Helper functions, with explicit scopes, moved from implicit namespace.
void VerticalSpineModel::trace(const tgStructureInfo& structureInfo, tgModel& model)
{
    std::cout << std::endl << "StructureInfo Trace:" << std::endl
    << structureInfo    << std::endl
    << "Model: "        << std::endl
    << model            << std::endl;
}

void VerticalSpineModel::addNodes(tgStructure& vertebra, ConfigVertebra& conf_vertebra)
{
    // right
    vertebra.addNode( getEdgeLength(conf_vertebra) / 2.0, 0, 0); // node 0
    // left
    vertebra.addNode( -getEdgeLength(conf_vertebra) / 2.0, 0, 0); // node 1
    // top
    vertebra.addNode(0, conf_vertebra.height, -getEdgeLength(conf_vertebra) / 2.0); // node 2
    // front
    vertebra.addNode(0, conf_vertebra.height, getEdgeLength(conf_vertebra) / 2.0); // node 3
    // middle
    vertebra.addNode(0, conf_vertebra.height/2, 0); // node 4

}

double VerticalSpineModel::getDensity(ConfigVertebra& conf_vertebra)
{
  // do we want an assert() here?
  // Remember that the mass here is distributed over the whole volume of
  // the vertebra. Assume that one vertebra is 4 rods.
  int num_rods = 4;
  double volume = num_rods * M_PI * pow(conf_vertebra.radius, 2) * conf_vertebra.leg_length;
  double density = conf_vertebra.mass / volume;
  return density;
}

double VerticalSpineModel::getEdgeLength(ConfigVertebra& conf_vertebra)
{
  // Do we want an assert here?
  // Leg length is hypotenuse with triangle with sides h/2 and e/2.
  double edge = 2 * sqrt( pow(conf_vertebra.leg_length, 2) - pow( conf_vertebra.height / 2, 2) );
  return edge;
}

void VerticalSpineModel::addRodPairs(tgStructure& vertebra, ConfigVertebra& conf_vertebra)
{
    vertebra.addPair(0, 4, "rod_" + conf_vertebra.vertebra_name);
    vertebra.addPair(1, 4, "rod_" + conf_vertebra.vertebra_name);
    vertebra.addPair(2, 4, "rod_" + conf_vertebra.vertebra_name);
    vertebra.addPair(3, 4, "rod_" + conf_vertebra.vertebra_name);
}

// DEPRECATED AS OF 2016-03-11
void VerticalSpineModel::addSegments(tgStructure& spine, const tgStructure& vertebra, 
				     double edge, size_t segment_count)
{
    //const btVector3 offset(0, 0, -edge * 1.15);
    //const btVector3 offset(0, 7.5, 0);
    const btVector3 offset(0, conf_spine.vertebra_separation, 0);
    // For segment_count many more vertebrae...
    for (size_t i = 1; i < segment_count; i++)
    {
        // Make a copy of the given vertebra object 
        tgStructure* const new_vertebra = new tgStructure(vertebra);
	// Name the new vertebra
        new_vertebra -> addTags(tgString("segment", i + 1));
	// Move it into its new location.
        new_vertebra -> move((i + 1) * offset);
        // Add this new vertebra to the spine
        spine.addChild(new_vertebra);
    }
        
}

void VerticalSpineModel::addVertebra(tgStructure& spine,
				     ConfigVertebra& conf_vertebra,
				     size_t vertebra_number)
{
    // we have to pull out the base vertebra here to copy it.
    // TO-DO: Why can't we just make a new object?
    // What does this call to new tgStructure(something) really do?
    std::vector<tgStructure*> spine_vertebrae = spine.getChildren();
    // Find the first vertebra
    // HACK FOR NOW: just pick the first element, we know that's the base.
    tgStructure* const new_vertebra = new tgStructure( *(spine_vertebrae[0]) );

    // TO-DO: REMOVE THE OLD TAG FROM THE FIRST VERTEBRA!
    // Need to remove "segment1".
    // DO WE ALSO NEED TO REMOVE OTHER OLD STUFF?

    // Add the nodes and pairs to this new vertebra
    addNodes( *new_vertebra, conf_vertebra);
    addRodPairs( *new_vertebra, conf_vertebra);

    // Move it into place.
    // Remember that the vertical direction is dimension 2 in Bullet Physics.
    new_vertebra -> move( btVector3(0.0,
		      (vertebra_number - 1) * conf_spine.vertebra_separation, 0.0) );

    // Tag it with the segment number
    new_vertebra -> addTags( tgString("segment", vertebra_number) );

    // Add the created vertebra to the rest of the spine.
    spine.addChild(new_vertebra);
}

// Add muscles that connect the segments
void VerticalSpineModel::addMuscles(tgStructure& spine)
{
    const std::vector<tgStructure*> children = spine.getChildren();
    for (size_t i = 1; i < children.size(); i++)
    {
        tgNodes n0 = children[i-1]->getNodes();
        tgNodes n1 = children[i  ]->getNodes();
                
        // vertical muscles
        spine.addPair(n0[0], n1[0], "vertical muscle a");
        spine.addPair(n0[1], n1[1], "vertical muscle b");
        spine.addPair(n0[2], n1[2], "vertical muscle c");
        spine.addPair(n0[3], n1[3], "vertical muscle d");

        // saddle muscles
        spine.addPair(n0[2], n1[1], tgString("saddle muscle seg", i-1));
        spine.addPair(n0[3], n1[1], tgString("saddle muscle seg", i-1));
        spine.addPair(n0[2], n1[0], tgString("saddle muscle seg", i-1));
        spine.addPair(n0[3], n1[0], tgString("saddle muscle seg", i-1));
    }
}

void VerticalSpineModel::mapMuscles(VerticalSpineModel::MuscleMap& muscleMap,
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

/***************************************
 * The primary functions., called from other classes.
 **************************************/
void VerticalSpineModel::setup(tgWorld& world)
{

    // Flag for debugging output: set to 1 to get more information output
    // to the command line when the app is run
    bool debug_flag = 0;

    // A bit of debugging about this spine's geometry and shape.
    if( debug_flag ){
      std::cout << "passive vertebra height: " << conf_passive_vertebra.height
		<< std::endl;
      std::cout << "passive vertebra edge length: " <<
	getEdgeLength(conf_passive_vertebra) << std::endl;
    }

    // This is the container for the whole spine object,
    // including all rigid bodies and all cables.
    tgStructure spine;
  
    // Create the first spine vertebra. This one is fixed to the ground,
    // and is non-moving since it has mass = 0.
    tgStructure base_vertebra;
    addNodes(base_vertebra, conf_base_vertebra);
    addRodPairs(base_vertebra, conf_base_vertebra);
    
    // Remember that the second dimension here is vertical.
    // (Y is the vertical, not Z.)
    base_vertebra.move(btVector3(0.0, conf_spine.base_vertical_offset, 0));
    
    // Tag this vertebra as the first segment in the spine   
    base_vertebra.addTags(tgString("segment", 1));
    // Apparently we need a const pointer here. Hrrmmmm... Drew 2016-03-10
    tgStructure* const base_vertebra_copy = new tgStructure(base_vertebra);
    spine.addChild(base_vertebra_copy);

    // Add the rest of the vertebrae! As of 2016-03-11, have four more.
    // The second and 4th vertebrae are actuated.
    addVertebra(spine, conf_active_vertebra, 2);
    addVertebra(spine, conf_passive_vertebra, 3);
    addVertebra(spine, conf_active_vertebra, 4);
    addVertebra(spine, conf_passive_vertebra, 5);

    
    addMuscles(spine);

    // Create the build spec that uses tags to turn the structure into a real model.
    // This is where the tagged pairs are matched up with an actual geometric object.
    tgBuildSpec spec;

    // For the base
    const tgRod::Config rodConfigBase(conf_base_vertebra.radius,
				      getDensity(conf_base_vertebra),
				      conf_base_vertebra.friction,
				      conf_base_vertebra.rollFriction,
				      conf_base_vertebra.restitution);

    // For the passive vertebra
    const tgRod::Config rodConfigPassive(conf_passive_vertebra.radius,
				      getDensity(conf_passive_vertebra),
				      conf_passive_vertebra.friction,
				      conf_passive_vertebra.rollFriction,
				      conf_passive_vertebra.restitution);

    // For the active vertebra
    const tgRod::Config rodConfigActive(conf_active_vertebra.radius,
				      getDensity(conf_active_vertebra),
				      conf_active_vertebra.friction,
				      conf_active_vertebra.rollFriction,
				      conf_active_vertebra.restitution);

    // Add to the build specification: these tgRod::Config objects matched up
    // with the vertebra name. This correlates to what's used in addRodPairs.
    spec.addBuilder("rod_" + conf_base_vertebra.vertebra_name,
		    new tgRodInfo(rodConfigBase));
    spec.addBuilder("rod_" + conf_passive_vertebra.vertebra_name,
		    new tgRodInfo(rodConfigPassive));
    spec.addBuilder("rod_" + conf_active_vertebra.vertebra_name,
		    new tgRodInfo(rodConfigActive));


    // set muscle (string) parameters
    // @todo replace acceleration constraint with tgKinematicActuator if needed...
    tgSpringCableActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension,
					 c.hist, c.maxTens, c.targetVelocity);
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    
    // Create the structureInfo
    tgStructureInfo structureInfo(spine, spec);

    // Debugging: print out the tgStructure of the spine and its children
    std::cout << spine << std::endl;
    trace(structureInfo, *this);

    // Use the structureInfo to build this model
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the models (e.g. muscles)
    // that we want to control.    
    allMuscles = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());
    mapMuscles(muscleMap, *this, m_segments);

    // Let's see what type of objects are inside the spine.
    std::vector<tgModel*> all_children = getDescendants();
    // Pick out the tgBaseRigid objects
    std::vector<tgBaseRigid*> all_tgBaseRigid = tgCast::filter<tgModel, tgBaseRigid>(all_children);

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
    notifySetup();
    tgModel::setup(world);
}

void VerticalSpineModel::step(double dt)
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
    
const std::vector<tgSpringCableActuator*>&
VerticalSpineModel::getMuscles (const std::string& key) const
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

const std::vector<tgSpringCableActuator*>& VerticalSpineModel::getAllMuscles() const
{
    return allMuscles;
}

