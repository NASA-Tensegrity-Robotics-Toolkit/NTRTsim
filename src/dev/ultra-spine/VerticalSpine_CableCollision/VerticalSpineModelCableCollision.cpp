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
 * @author Brian Tietz, Drew Sabelhaus, Michael Fanton, ChanWoo Yang
 * $Id$
 */

// This module
#include "VerticalSpineModelCableCollision.h"
// This library
#include "core/tgCast.h"
//#include "core/tgBasicActuator.h"
//replaced
#include "core/tgSpringCableActuator.h"
//added
#include "core/tgSphere.h"
#include "core/tgString.h"
#include "tgcreator/tgBuildSpec.h"
//#include "tgcreator/tgBasicActuatorInfo.h"
//added
#include "tgcreator/tgBasicContactCableInfo.h"
#include "tgcreator/tgRigidAutoCompound.h"
#include "tgcreator/tgRodInfo.h"
//added
#include "tgcreator/tgSphereInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgUtil.h"
// The Bullet Physics library
#include "btBulletDynamicsCommon.h"
// The C++ Standard Library
#include <iostream>
#include <stdexcept>

// @todo move hard-coded parameters into config

//m_segments(segments),
//tgModel()
VerticalSpineModelCableCollision::VerticalSpineModelCableCollision(size_t segments) :
    BaseSpineModelLearning(segments)
{
}

VerticalSpineModelCableCollision::~VerticalSpineModelCableCollision()
{
}
/**
 * Anonomous namespace for config struct. This makes changing the parameters
 * of the model much easier (they're all at the top of this file!).
 */
namespace
{
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
     2452.0,        // pretension
     0,			// History logging (boolean)
     100000*1000000,   // maxTens
     10000,    // targetVelocity
      
  };



  // Helper functions, with explicit scopes, moved from implicit namespace.
  void trace(const tgStructureInfo& structureInfo, tgModel& model)
  {
    std::cout << "StructureInfo:" << std::endl
	      << structureInfo    << std::endl
	      << "Model: "        << std::endl
	      << model            << std::endl;
  }

  void addNodes(tgStructure& tetra, double edge, double height)
  {
    // right
    tetra.addNode( c.edge / 2.0, 0, 0, "sphere"); // node 0
    // left
    tetra.addNode( -c.edge / 2.0, 0, 0, "sphere"); // node 1
    // top
    tetra.addNode(0, c.height, -edge / 2.0, "sphere"); // node 2
    // front
    tetra.addNode(0, c.height, edge / 2.0, "sphere"); // node 3
    // middle
    tetra.addNode(0, c.height/2, 0, "sphere"); // node 4

    //right holder
    tetra.addNode(c.edge / 2.0, 0, 0.5, "sphere"); // node 5
    tetra.addNode( c.edge / 2.0, 0, -0.5, "sphere"); // node 6
    tetra.addNode( 0.5+c.edge / 2.0, 0, 0.5, "sphere"); // node 7
    tetra.addNode( 0.5+c.edge / 2.0, 0, -0.5, "sphere"); // node 8
    //left holder
    tetra.addNode(-c.edge / 2.0, 0, 0.5, "sphere"); // node 9
    tetra.addNode(-c.edge / 2.0, 0, -0.5, "sphere"); // node 10
    tetra.addNode(-0.5 + -c.edge / 2.0, 0, 0.5, "sphere"); // node 11
    tetra.addNode(-0.5 + -c.edge / 2.0, 0, -0.5, "sphere"); // node 12
    //top holder
    tetra.addNode(0.5, c.height, -edge / 2.0, "sphere"); // node 13
    tetra.addNode(-0.5, c.height, -edge / 2.0, "sphere"); // node 14
    tetra.addNode(0.5, c.height, -0.5 + -edge / 2.0, "sphere"); // node 15
    tetra.addNode(-0.5, c.height, -0.5 + -edge / 2.0, "sphere"); // node 16
    //front holder
    tetra.addNode(0.5, c.height, edge / 2.0, "sphere"); // node 17
    tetra.addNode(-0.5, c.height, edge / 2.0, "sphere"); // node 18
    tetra.addNode(0.5, c.height, 0.5 + edge / 2.0, "sphere"); // node 19
    tetra.addNode(-0.5, c.height, 0.5 + edge / 2.0, "sphere"); // node 20
  }
  
  void addNodesB(tgStructure& tetra, double edge, double height)
  {
    
    // right
    tetra.addNode( c.edge / 2.0, 0, 0); // node 0
    // left
    tetra.addNode( -c.edge / 2.0, 0, 0); // node 1
    // top
    tetra.addNode(0, c.height, -edge / 2.0); // node 2
    // front
    tetra.addNode(0, c.height, edge / 2.0); // node 3
    // middle
    tetra.addNode(0, c.height/2, 0); // node 4
    /*
    //right holder
    tetra.addNode(c.edge / 2.0, 0, 0.5, "sphereB"); // node 5
    tetra.addNode( c.edge / 2.0, 0, -0.5, "sphereB"); // node 6
    tetra.addNode( 0.5+c.edge / 2.0, 0, 0.5, "sphereB"); // node 7
    tetra.addNode( 0.5+c.edge / 2.0, 0, -0.5, "sphereB"); // node 8
    //left holder
    tetra.addNode(-c.edge / 2.0, 0, 0.5, "sphereB"); // node 9
    tetra.addNode(-c.edge / 2.0, 0, -0.5, "sphereB"); // node 10
    tetra.addNode(-0.5 + -c.edge / 2.0, 0, 0.5, "sphereB"); // node 11
    tetra.addNode(-0.5 + -c.edge / 2.0, 0, -0.5, "sphereB"); // node 12
    //top holder
    tetra.addNode(0.5, c.height, -edge / 2.0, "sphereB"); // node 13
    tetra.addNode(-0.5, c.height, -edge / 2.0, "sphereB"); // node 14
    tetra.addNode(0.5, c.height, -0.5 + -edge / 2.0, "sphereB"); // node 15
    tetra.addNode(-0.5, c.height, -0.5 + -edge / 2.0, "sphereB"); // node 16
    //front holder
    tetra.addNode(0.5, c.height, edge / 2.0, "sphereB"); // node 17
    tetra.addNode(-0.5, c.height, edge / 2.0, "sphereB"); // node 18
    tetra.addNode(0.5, c.height, 0.5 + edge / 2.0, "sphereB"); // node 19
    tetra.addNode(-0.5, c.height, 0.5 + edge / 2.0, "sphereB"); // node 20
    */
  }

  void addPairs(tgStructure& tetra)
  {
    tetra.addPair(0, 4, "rod");
    tetra.addPair(1, 4, "rod");
    tetra.addPair(2, 4, "rod");
    tetra.addPair(3, 4, "rod");

    for (double i = 0; i<=3; i++)
      {
	tetra.addPair(i,i*4+5, "holder");
	tetra.addPair(i,i*4+6, "holder");
        tetra.addPair(i*4+5,i*4+7, "holder");
	tetra.addPair(i*4+6,i*4+8, "holder");
	tetra.addPair(i*4+7,i*4+8, "holder");
      }
    /*
    //holder
    tetra.addPair(0,5, "holder");
    tetra.addPair(0,6, "holder");
    tetra.addPair(5,7, "holder");
    tetra.addPair(6,8, "holder");
    tetra.addPair(7,8, "holder");
    */
  }
    
  void addPairsB(tgStructure& tetra)
  {
    tetra.addPair(0, 4, "rodB");
    tetra.addPair(1, 4, "rodB");
    tetra.addPair(2, 4, "rodB");
    tetra.addPair(3, 4, "rodB");
    /*
    for (double i = 0; i<=3; i++)
      {
	tetra.addPair(i,i*4+5, "holderB");
	tetra.addPair(i,i*4+6, "holderB");
	tetra.addPair(i*4+6,i*4+8, "holderB");
        tetra.addPair(i*4+5,i*4+7, "holderB");
	tetra.addPair(i*4+7,i*4+8, "holderB");
      }
    
    //holder
    tetra.addPair(0,5, "holderB");
    tetra.addPair(0,6, "holderB");
    tetra.addPair(5,7, "holderB");
    tetra.addPair(6,8, "holderB");
    tetra.addPair(7,8, "holderB");
    */
  }

  void addSegments(tgStructure& snake, const tgStructure& tetra, 
		   double edge, size_t segmentCount)
  {
    //const btVector3 offset(0, 0, -edge * 1.15);
    const btVector3 offset(0, 7.5, 0);
    for (size_t i = 1; i < segmentCount; ++i)
      {
        tgStructure* const t = new tgStructure(tetra);
        t->addTags(tgString("segment", i + 1));
        t->move((i + 1) * offset);
        // Add a child to the snake
        snake.addChild(t);
      }
        
  }

  // Add muscles that connect the segments
  // Underactuated Model Structure
  void addMuscles(tgStructure& snake)
  {
    const std::vector<tgStructure*> children = snake.getChildren();
    for (size_t i = 1; i < children.size(); ++i)
      {
        tgNodes vn0 = children[0]->getNodes();
        tgNodes vn1 = children[i  ]->getNodes();
                
        // vertical muscles
        snake.addPair(vn0[0], vn1[0], "vertical muscle a");
        snake.addPair(vn0[1], vn1[1], "vertical muscle b");
        snake.addPair(vn0[2], vn1[2], "vertical muscle c");
        snake.addPair(vn0[3], vn1[3], "vertical muscle d");

	tgNodes sn0 = children[i-1]->getNodes();
        tgNodes sn1 = children[i  ]->getNodes();
        // saddle muscles
        snake.addPair(sn0[2], sn1[1], tgString("saddle muscle seg", i-1));
        snake.addPair(sn0[3], sn1[1], tgString("saddle muscle seg", i-1));
        snake.addPair(sn0[2], sn1[0], tgString("saddle muscle seg", i-1));
        snake.addPair(sn0[3], sn1[0], tgString("saddle muscle seg", i-1));
      }
  }

  void mapMuscles(VerticalSpineModelCableCollision::MuscleMap& muscleMap,
		  tgModel& model, size_t segmentCount)
  {
    // create names for muscles (for getMuscles function)
    
    // vertical muscles
    muscleMap["vertical a"] = model.find<tgSpringCableActuator>("vertical muscle a");
    muscleMap["vertical b"] = model.find<tgSpringCableActuator>("vertical muscle b");
    muscleMap["vertical c"] = model.find<tgSpringCableActuator>("vertical muscle c");
    muscleMap["vertical d"] = model.find<tgSpringCableActuator>("vertical muscle d");
        
    // saddle muscles
    for (size_t i = 1; i < segmentCount ; ++i)
      {
        muscleMap[tgString("saddle", i-1)] = model.find<tgSpringCableActuator>(tgString("saddle muscle seg", i-1));
            
      }
  }
} // end namespace

/***************************************
 * The primary functions., called from other classes.
 **************************************/
void VerticalSpineModelCableCollision::setup(tgWorld& world)
{
    // debugging output: edge and height length
    std::cout << "edge: " << c.edge << "; height: " << c.height << std::endl;
    
    // Create the first fixed snake segment
    // @todo move these hard-coded parameters into config
    tgStructure tetraB;
    addNodesB(tetraB, c.edge, c.height);
    addPairsB(tetraB);
    tetraB.move(btVector3(0.0, 2, 0));
    
    // Create our snake segments
    tgStructure snake;
    
    // add 1st child to snake
    tgStructure* const tB = new tgStructure(tetraB);
    snake.addChild(tB);
    tB->addTags(tgString("segment", 1));
    
    // Create the first non-fixed tetrahedra
    tgStructure tetra;
    addNodes(tetra, c.edge, c.height);
    addPairs(tetra);
    
    // Move the first tetrahedra
    // @todo move these hard-coded parameters into config
    tetra.move(btVector3(0.0, -6, 0));
    
    // add rest of segments using original tetra configuration
    addSegments(snake, tetra, c.edge, m_segments);
    
    addMuscles(snake);

    // Create the build spec that uses tags to turn the structure into a real model
    // Note: This needs to be high enough or things fly apart...
    
    // length of inner strut = 12.25 cm
    // m = 1 kg
    // volume of 1 rod = 9.62 cm^3
    // total volume = 38.48 cm^3
    //const double density = 1/38.48; = 0.026 // kg / length^3 - see app for length
    const tgRod::Config rodConfigA(c.radius, c.densityA, c.friction, 
				  c.rollFriction, c.restitution);
    const tgRod::Config rodConfigB(c.radius, c.densityB, c.friction, 
				  c.rollFriction, c.restitution);
    //holder
    const tgRod::Config holderConfigA(0.15, c.densityA, c.friction,
				    c.rollFriction, c.restitution);
    const tgRod::Config holderConfigB(0.15, c.densityB, c.friction,
				    c.rollFriction, c.restitution);

    //welding holders
    const tgSphere::Config weldingConfigA(0.25, c.densityA);
    //const tgSphere::Config weldingConfigB(0.3, c.densityA);

    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfigA));
    spec.addBuilder("rodB", new tgRodInfo(rodConfigB));

    // holder---------------------------------------------
    spec.addBuilder("holder", new tgRodInfo(holderConfigA));
    spec.addBuilder("holderB", new tgRodInfo(holderConfigB));    
    //----------------------------------------------------

    // welding
    spec.addBuilder("sphere", new tgSphereInfo(weldingConfigA));
    //spec.addBuilder("sphereB", new tgSphereInfo(weldingConfigB));
    //-------------

    // set muscle (string) parameters
    // @todo replace acceleration constraint with tgKinematicActuator if needed...
    tgSpringCableActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension,
					       false, c.maxTens, c.targetVelocity, 1.0,0.1,0.0);
    spec.addBuilder("muscle", new tgBasicContactCableInfo(muscleConfig));
    //spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(snake, spec);
    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the models (e.g. muscles)
    // that we want to control.    
    //m_allMuscles = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());
    //m_allSegments = tgCast::filter<tgModel,tgRod> (getDescendants());
    m_allMuscles = this->find<tgSpringCableActuator> ("muscle");
    m_allSegments = this->find<tgModel> ("segment");
    
    mapMuscles(m_muscleMap, *this, m_segments);

    trace(structureInfo, *this);
    /*
    // Actually setup the children
    tgModel::setup(world);*/
    BaseSpineModelLearning::setup(world);
}

void VerticalSpineModelCableCollision::teardown()
{
  BaseSpineModelLearning::teardown();
}

void VerticalSpineModelCableCollision::step(double dt)
{
    if (dt < 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
      /*
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        // Step any children
        tgModel::step(dt);
      */
      BaseSpineModelLearning::step(dt);
    }
}
/*    
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

*/
