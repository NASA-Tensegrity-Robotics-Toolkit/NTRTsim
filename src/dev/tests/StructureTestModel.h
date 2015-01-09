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

#ifndef CONNECTOR_TEST_MODEL_H
#define CONNECTOR_TEST_MODEL_H

#include "core/tgModel.h" 
#include "core/tgBasicActuator.h"

#include "core/tgSubject.h"
#include "core/tgModelVisitor.h"

#include "btBulletDynamicsCommon.h"
#include <iostream>
#include <set>

#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRigidAutoCompound.h"
#include "tgcreator/tgBuildSpec.h"

#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"


class StructureTestModel: public tgSubject<StructureTestModel>, public tgModel
{
public: 

    StructureTestModel(std::string name) : tgModel(name) 
    {
        
    }

    ~StructureTestModel()
    {
    }
    
    /** @todo Get rid of this. */
    virtual void setup(tgWorld& world)
    {
        // This is basically a manual setup of a model. There are things that do this for us (@todo: reference the things that do this for us)
        
        const double density = 0.9;
        const double radius  = 0.3;
        const tgRod::Config rodConfig(radius, density);
        
        double stiffness = 100000;
        double dampening = stiffness * 0.001;
        
        tgBasicActuator::Config muscleConfig(stiffness, dampening);
        
        tgStructure structure;
        
        structure.addNode(0,0,0);
        structure.addNode(0,3,0);
        structure.addNode(2,5,0);
        structure.addNode(-2,5,0);
        
        structure.addNode(0, 10, 0);
        structure.addNode(0, 7, 0);
        structure.addNode(0, 5, 2);
        structure.addNode(0, 5,-2);
        
        structure.addPair(0, 1, "rod");
        structure.addPair(1, 2, "rod");
        structure.addPair(1, 3, "rod");

        structure.addPair(4, 5, "rod");
        structure.addPair(5, 6, "rod");
        structure.addPair(5, 7, "rod");

        structure.addPair(2, 6, "muscle");
        structure.addPair(3, 7, "muscle");
        structure.addPair(2, 7, "muscle");
        structure.addPair(3, 6, "muscle");
    
        tgBuildSpec spec;
        spec.addBuilder("rod", new tgRodInfo(rodConfig));
        spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
        
        tgStructureInfo structureInfo(structure, spec);
        structureInfo.buildInto(*this, world);


        // Note: all these steps are done for you by structureInfo.buildInto()
        /*
        structureInfo.initRigidInfo();
        structureInfo.autoCompoundRigids();
        
        structureInfo.initConnectorInfo();
        structureInfo.chooseConnectorRigids();

        structureInfo.initRigidBodies(world);
        structureInfo.initConnectors(world); // Note: Muscle2Ps won't show up yet -- they need to be part of a model to have rendering...
        */
        
        
        //structureInfo.createRigidModels(world);
        //structureInfo.createConnectorModels(world);
        
        std::cout << "StructureInfo:" << std::endl;
        std::cout << structureInfo << std::endl;

    }
    
    virtual void step(double dt)
    {
        //btTransform trans;
        //m_rod->getRigidBody()->getMotionState()->getWorldTransform(trans);
        //std::cout << "rod height: " << trans.getOrigin().getY() << std::endl;
        
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        
        tgModel::step(dt);  // Step any children
    }
    
    virtual void onVisit(tgModelVisitor& r)
    {
        // Example: m_rod->getRigidBody()->dosomething()...
        
        //m_testString->onRender(r);
        
        tgModel::onVisit(r);
        //std::cout << "      StructureTestModel has special rendering requirements!" << std::endl;
    }
    
    //Does this make myModel an observer as well??
    void changeMuscle (double length)
    {
        //Do we need error checking here too?
        //newString->setRestLength(length); // commented out for testing
    }
    
private:
    tgRodInfo* m_rod;    
    tgRodInfo* m_rod2;
    
    //tgBasicActuator* newString;
    tgModel* m_testString;
};

#endif
