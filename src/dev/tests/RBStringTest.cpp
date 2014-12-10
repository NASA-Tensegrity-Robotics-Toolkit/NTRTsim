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

#include "RBStringTest.h"

#include "btBulletDynamicsCommon.h"
#include <iostream>

#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgConnectorInfo.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRigidAutoCompound.h"
#include "tgcreator/tgBuildSpec.h"

#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgUtil.h"
#include "core/tgString.h"
#include "tgcreator/tgNode.h"

#include "core/tgCast.h"

RBStringTest::Config::Config( int segments,
                const tgRod::Config& rodConf,
                const tgBasicActuator::Config& stringConf,
                double minTotalLength) :
m_segments(segments),
m_rodConfig(rodConf),
m_stringConfig(stringConf),
m_minTotalLength(minTotalLength)
{
}

RBStringTest::RBStringTest(tgNode* start, tgNode* end, const RBStringTest::Config& config) :
    m_pStartNode(start),
    m_pEndNode(end),
    m_config(config),
    tgModel() 
{
}

void RBStringTest::setup(tgWorld& world)
{
    
    // todo - figure out a way to change only the one parameter with less code
    const double stiffness = m_config.m_stringConfig.stiffness;
    const double damping = m_config.m_stringConfig.damping;
    const double pretension = 0.0;
    tgBasicActuator::Config muscleConfig1(stiffness, damping, pretension, false, 1000.0, 100.0, 0.1, 0.1, -M_PI / 2.0);
    tgBasicActuator::Config muscleConfig2(stiffness, damping, pretension, false, 1000.0, 100.0, 0.1, 0.1, M_PI / 2.0);
    tgBasicActuator::Config muscleConfig3(stiffness, damping, pretension, false, 1000.0, 100.0, 0.1, 0.1, M_PI);
    tgBasicActuator::Config muscleConfig4(stiffness, damping, pretension, false, 1000.0, 100.0, 0.1, 0.1, 0);
    
    // Calculations for the flemons spine model
    double v_size = m_config.m_minTotalLength / (double) m_config.m_segments;
    
    // tgNode is a subclass, so this should be fine
    btVector3 buildVec = (*m_pEndNode - *m_pStartNode);
    
    double spacing = buildVec.length() / (double) m_config.m_segments;
    // After this buildVec stays normalized.
    btVector3 offset = buildVec.normalize() * spacing; 
    
    // Create the tetrahedra
    tgStructure tetra;
    
    tetra.addNode(0,0,0);  // Bottom
    tgNode endNode(v_size * buildVec); //Fancy upcast
    tetra.addNode(endNode); // Top
    
    tetra.addPair(0,1, "a rod");

    // Move the first one into position
    tetra.move(*m_pStartNode);
    

    // Create our snake segments - these will be tgModels with
    // tgRods as children
    tgStructure snake;
    
    for(int i = 0; i < m_config.m_segments; i++) {
        // @todo: the snake is a temporary variable -- will its destructor be called? If not, where do we delete its children?
        tgStructure* t = new tgStructure(tetra);
        t->addTags(tgString("segment num", i + 1));
        t->move((i + 1)*offset);
        snake.addChild(t); // Add a child to the snake
    }

    // Add muscles that connect the segments
    std::vector<tgStructure*> children = snake.getChildren();
    for(int i = 1; i < children.size(); i++) {
        tgNodes n0 = children[i-1]->getNodes();
        tgNodes n1 = children[i]->getNodes();
        
        snake.addPair(n0[1], n1[0], tgString("muscle seg", i-1) + tgString(" seg", i));
        snake.addPair(n0[1], n1[0], tgString("muscle2 seg", i-1) + tgString(" seg", i));
        snake.addPair(n0[1], n1[0], tgString("muscle3 seg", i-1) + tgString(" seg", i));
        snake.addPair(n0[1], n1[0], tgString("muscle4 seg", i-1) + tgString(" seg", i));
    }

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(m_config.m_rodConfig));
    
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig1));
    spec.addBuilder("muscle2", new tgBasicActuatorInfo(muscleConfig2));
    spec.addBuilder("muscle3", new tgBasicActuatorInfo(muscleConfig3));
    spec.addBuilder("muscle4", new tgBasicActuatorInfo(muscleConfig4));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(snake, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the models (e.g. muscles) that we want to control. 
    allMuscles = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());
    
    // Debug printing
    std::cout << "StructureInfo:" << std::endl;
    std::cout << structureInfo << std::endl;

    std::cout << "Model: " << std::endl;
    std::cout << *this << std::endl;
        

}

void RBStringTest::step(double dt)
{
    // Notify observers (controllers) of the step so that they can take action
    notifyStep(dt);

    tgModel::step(dt);  // Step any children

}
