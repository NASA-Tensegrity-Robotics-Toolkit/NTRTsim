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

#ifndef BUILD_TEST_MODEL_H
#define BUILD_TEST_MODEL_H

#include "core/tgModel.h" 
#include "core/tgBasicActuator.h"
#include "tgcreator/tgNodes.h"
#include "core/tgSubject.h"
#include "core/tgModelVisitor.h"

#include "btBulletDynamicsCommon.h"
#include <iostream>
#include <set>
#include <vector>

#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgRigidAutoCompound.h"
//#include "tgcreator/tgRigidCreator.h"

class BuildTestModel: public tgSubject<BuildTestModel>, public tgModel
{
public: 

    BuildTestModel(std::string name) : tgModel(name) 
    {
        
    }

    ~BuildTestModel()
    {
    }
    
    virtual void setup(tgWorld& world)
    {
        // This is basically a manual setup of a model. There are things that do this for us (@todo: reference the things that do this for us)
        
        const double density = 0.9;
        const double radius  = 0.3;
        const tgRod::Config rodConfig(radius, density);
        
        tgNodes nodes;
        nodes.addNode(0,0,0);
        nodes.addNode(0,5,0);
        nodes.addNode(5,8,0);
        nodes.addNode(5,5,0);
        
        tgPairs pairs;
        pairs.addPair(nodes.pair(0, 1), "rod");
        pairs.addPair(nodes.pair(1, 2), "rod");
        pairs.addPair(nodes.pair(2, 3), "rod");
        pairs.addPair(nodes.pair(3, 0), "rod");
        
        m_rod = new tgRodInfo(rodConfig, pairs[0]);
        m_rod2 = new tgRodInfo(rodConfig, pairs[1]);
        // @todo: This doesn't compound correctly if rod4 isn't there?? rod4 does if rod3 isn't... probably an off-by-one error in compounding...
        tgRodInfo* rod3 = new tgRodInfo(rodConfig, tgPair(nodes[2], nodes[3], "rod")); 
        //tgRodInfo* rod4 = new tgRodInfo(rodConfig, pairs[3]);
        
        // Start with a set of tgRigidInfo*. A tgStructureInfo will also make this available.
        std::vector<tgRigidInfo*> rigids;
        rigids.push_back(m_rod);
        rigids.push_back(m_rod2);
        rigids.push_back(rod3);
        //rigids.insert(rod4);

        // Create compound rigids out of rigids that share nodes
        tgRigidAutoCompound autoCompound(rigids);
        autoCompound.execute(); // After this we have all rigids grouped (even if they're in their own group)

        // not using this right now...
        //tgBuildSpec spec;
        //spec.addBuilder("rod", new tgRodInfo(rodConfig));  // @todo: should probably have tgRigidBuilder be abstract and use something like tgRodBuilder

        

        std::vector<tgRigidInfo*>::iterator it;
        for(it = rigids.begin(); it != rigids.end(); it++) {
            (*it)->createModel(world);
        }

        // Create a btRigidBody for each rigid and give ownership to the corresponding tgRigidInfo 
        //tgRigidCreator rigidCreator(world);
        //rigidCreator.buildRigidBodies(rigids);

        /*
        rigidCreator.createRigidBodies(m_rod);
        rigidCreator.createRigidBodies(m_rod2);
        rigidCreator.createRigidBodies(rod3);
        */
        /*
        btenRigidBuilder rb = btenRigidBuilder(*(world->getDynamicsWorld()));

        btenMuscle2PBuilder mb = btenMuscle2PBuilder((world->getDynamicsWorld()));
        
        btenMuscle2PBuilder::Muscle2PConfig muscleConfig;
        muscleConfig.elasticity = 1000;
        muscleConfig.dampening = 10;
        
        rb.addRigid(*m_rod);
        rb.addRigid(*m_rod2);
        
        rb.init();
        
        btVector3* cp1 = new btVector3(0, 20, 0);
        btVector3* cp2 = new btVector3(10, 20, 0);
        
        newString = new tgBasicActuator(mb.createMuscle(*m_rod, *cp1, *m_rod2, *cp2, muscleConfig), "Muscle1");
        
        this->addChild(newString);

        // std::cout << "after initialize, rod is " << m_rod << std::endl;
        
        delete cp1;
        delete cp2;
        
        tgModel::setup(world); // Setup any children we might have...
        */

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
    
    virtual void onVisit(const tgModelVisitor& r) const
    {
        // Example: m_rod->getRigidBody()->dosomething()...
        
        //newString->onRender(r);
        
        //tgModel::onRender(r);
        //std::cout << "      BuildTestModel has special rendering requirements!" << std::endl;
    }
    
    void changeMuscle (double length)
    {
        //Do we need error checking here too?
        //newString->setRestLength(length); // commented out for testing
    }
    
private:
    tgRodInfo* m_rod;    
    tgRodInfo* m_rod2;
    
    tgBasicActuator* newString;
};

#endif
