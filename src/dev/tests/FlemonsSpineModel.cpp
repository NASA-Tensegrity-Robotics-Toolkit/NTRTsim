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

#include "FlemonsSpineModel.h"

#include "core/tgLinearString.h"

#include "btBulletDynamicsCommon.h"
#include <iostream>

#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgConnectorInfo.h"
#include "tgcreator/tgLinearStringInfo.h"
#include "tgcreator/tgRigidAutoCompound.h"
#include "tgcreator/tgBuildSpec.h"

#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgUtil.h"
#include "core/tgString.h"

#include "core/tgCast.h"

FlemonsSpineModel::FlemonsSpineModel(int segments) : 
    m_segments(segments),
    tgModel() 
{
}

void FlemonsSpineModel::setup(tgWorld& world)
{
    // Rod and Muscle configuration
    
    const double density = 4.2/300.0;  // Note: This needs to be high enough or things fly apart...
    const double radius  = 0.5;
    const tgRod::Config rodConfig(radius, density);
    
    tgLinearString::Config muscleConfig(1000, 10);
    
    // Calculations for the flemons spine model
    double v_size = 10.0;
    
    // Create the tetrahedra
    tgStructure tetra;

    tetra.addNode(0,0,0);  // center
    tetra.addNode(v_size, v_size, v_size);   // front
    tetra.addNode(v_size, -v_size, -v_size);   // right
    tetra.addNode(-v_size, v_size,-v_size); // back
    tetra.addNode(-v_size, -v_size, v_size); // left
    
    tetra.addPair(0,1, "front rod");
    tetra.addPair(0,2, "right rod");
    tetra.addPair(0,3, "back rod");
    tetra.addPair(0,4, "left rod");

    // Move the first one so we can create a longer snake. Or you could move the snake at the end, up to you. 
    tetra.move(btVector3(0.0,15.0,100.0));

    // Create our snake segments - these will be tgModels with
    // tgRods as children
    tgStructure snake;
    btVector3 offset(0,0,-v_size * 1.15); // @todo: there seems to be an issue with Muscle2P connections if the front of a tetra is inside the next one.
    for(int i = 0; i < m_segments; i++) {
        // @todo: the snake is a temporary variable -- will its destructor be called? If not, where do we delete its children?
        tgStructure* t = new tgStructure(tetra);
        t->addTags(tgString("segment num", i + 1));
        t->move((i + 1)*offset);
        snake.addChild(t); // Add a child to the snake
    }
    //conditionally compile for debugging 
    #if (1)
    // Add muscles that connect the segments
    // Tag the muscles with their segment numbers so CPGs can find
    // them.
    std::vector<tgStructure*> children = snake.getChildren();
    for(int i = 1; i < children.size(); i++) {
        tgNodes n0 = children[i-1]->getNodes();
        tgNodes n1 = children[i]->getNodes();
    #if (0) // Conditional compile for "regular" vs "saddle joints"
        snake.addPair(n0[1], n1[1], tgString("outer front muscle seg", i-1) + tgString(" seg", i));
        snake.addPair(n0[2], n1[2], tgString("outer right muscle seg", i-1) + tgString(" seg", i));
        snake.addPair(n0[3], n1[3], tgString("outer back muscle seg", i-1) + tgString(" seg", i));
        snake.addPair(n0[4], n1[4], tgString("outer top muscle seg", i-1) + tgString(" seg", i));
    #else
        // @todo rename tags
        snake.addPair(n0[4], n1[1], tgString("outer front muscle seg", i-1) + tgString(" seg", i));
        snake.addPair(n0[1], n1[4], tgString("outer right muscle seg", i-1) + tgString(" seg", i));
        snake.addPair(n0[2], n1[3], tgString("outer back muscle seg", i-1) + tgString(" seg", i));
        snake.addPair(n0[3], n1[2], tgString("outer top muscle seg", i-1) + tgString(" seg", i));   
    #endif
        snake.addPair(n0[2], n1[1], tgString("inner front muscle seg", i-1) + tgString(" seg", i));
        snake.addPair(n0[2], n1[4], tgString("inner right muscle seg", i-1) + tgString(" seg", i));
        snake.addPair(n0[3], n1[1], tgString("inner left muscle seg", i-1) + tgString(" seg", i));
        snake.addPair(n0[3], n1[4], tgString("inner back muscle seg", i-1) + tgString(" seg", i));

    }
    #endif
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    
    #if (1)
    spec.addBuilder("muscle", new tgLinearStringInfo(muscleConfig));
    #endif
    
    // Create your structureInfo
    tgStructureInfo structureInfo(snake, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the models (e.g. muscles) that we want to control. 
    
    allMuscles = tgCast::filter<tgModel, tgLinearString> (getDescendants());
    
    
    /*
    // Note that tags don't need to match exactly, we could create
    // supersets if we wanted to
    muscleMap["inner left"] = this->find<tgLinearString>("inner left muscle");
    muscleMap["inner right"] = this->find<tgLinearString>("inner right muscle");
    muscleMap["inner top"] = this->find<tgLinearString>("inner top muscle");
    muscleMap["outer left"] = this->find<tgLinearString>("outer left muscle");
    muscleMap["outer right"] = this->find<tgLinearString>("outer right muscle");
    muscleMap["outer top"] = this->find<tgLinearString>("outer top muscle");
    */
    // Debug printing
    /*
    std::cout << "StructureInfo:" << std::endl;
    std::cout << structureInfo << std::endl;
    */
    std::cout << "Model: " << std::endl;
    std::cout << *this << std::endl;
    
    // Showing the find functiont
    std::vector<tgRod*> rodSegments = tgCast::filter<tgModel, tgRod> (getDescendants());
    for(int i = 0; i < rodSegments.size(); i++) {
        std::cout << "Structure: " << *(rodSegments[i]) << std::endl;
    }
    
    allSegments = this->find<tgModel> ("segment");
    for(int i = 0; i < allSegments.size(); i++) {
        std::cout << "Segs: " << *(allSegments[i]) << std::endl;
    }
    
    // Actually setup the children
    tgModel::setup(world);

}

void FlemonsSpineModel::step(double dt)
{
    // Notify observers (controllers) of the step so that they can take action
    notifyStep(dt);
    
    // TestCode for COM
    #if (0)
    std::vector<double> COM = getSegmentCOM(0);
    
    std::cout << "X: " << COM[0] << " Y: " << COM[1] << " Z: " << COM[2]
                <<std::endl;
    #endif
    
    tgModel::step(dt);  // Step any children

}

const std::vector<double> FlemonsSpineModel::getSegmentCOM(const int n) const
{
    if(n < 0) { throw std::range_error("Negative segment number"); }
    if(n >= m_segments) { throw std::range_error(tgString("Segment number > ",  m_segments)); }
    
    std::vector<tgRod*> p_rods = tgCast::filter<tgModel, tgRod> (allSegments[n]->getDescendants());
    
    // Ensure our segments are being populated correctly
    assert( p_rods.size() > 0);
    
    btVector3 segmentCenterOfMass(0, 0, 0);
    double segmentMass = 0.0;
    for (std::size_t i = 0; i < p_rods.size(); i++)
    {
            const tgRod* const pRod = p_rods[i];
        assert(pRod != NULL);
            const double rodMass = pRod->mass();
        const btVector3 rodCenterOfMass = pRod->centerOfMass();
        segmentCenterOfMass += rodCenterOfMass * rodMass;
        segmentMass += rodMass;
    }
    
    // Check to make sure the rods actually had mass
    assert(segmentMass > 0.0);
    
    segmentCenterOfMass /= segmentMass;

    // Copy to the result std::vector
    std::vector<double> result(3);
    for (size_t i = 0; i < 3; ++i) { result[i] = segmentCenterOfMass[i]; }
    
    return result;
}

void FlemonsSpineModel::changeMuscle (double length, double dt)
{
    for( int i = 0; i < allMuscles.size(); i++){
        allMuscles[i]->setRestLength(length, dt);
    }
}

