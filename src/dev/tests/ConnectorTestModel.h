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
#include "tgcreator/tgNodes.h"
#include "core/tgSubject.h"
#include "core/tgModelVisitor.h"

#include "btBulletDynamicsCommon.h"
#include <iostream>
#include <vector>
#include <set>

#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRigidAutoCompound.h"


class ConnectorTestModel: public tgSubject<ConnectorTestModel>, public tgModel
{
public: 

    ConnectorTestModel(std::string name) : tgModel(name) 
    {
        
    }

    ~ConnectorTestModel()
    {
    }
    
    virtual void setup(tgWorld& world)
    {
        // This is basically a manual setup of a model. There are things that do this for us (@todo: reference the things that do this for us)
        
        const double density = 0.9;
        const double radius  = 0.3;
        const tgRod::Config rodConfig(radius, density);
        
        tgBasicActuator::Config muscleConfig(1000, 10);
        
        tgNodes nodes;
        nodes.addNode(0,0,0);
        nodes.addNode(0,3,0);
        nodes.addNode(2,5,0);
        nodes.addNode(-2,5,0);
        
        nodes.addNode(0, 10, 0);
        nodes.addNode(0, 7, 0);
        nodes.addNode(0, 5, 2);
        nodes.addNode(0, 5,-2);
        
        tgPairs pairs;
        pairs.addPair(nodes.pair(0, 1), "rod");
        pairs.addPair(nodes.pair(1, 2), "rod");
        pairs.addPair(nodes.pair(1, 3), "rod");

        pairs.addPair(nodes.pair(4, 5), "rod");
        pairs.addPair(nodes.pair(5, 6), "rod");
        pairs.addPair(nodes.pair(5, 7), "rod");


        tgPairs musclePairs;
        musclePairs.addPair(nodes.pair(2, 6), "muscle");
        musclePairs.addPair(nodes.pair(3, 7), "muscle");
        musclePairs.addPair(nodes.pair(2, 7), "muscle");
        musclePairs.addPair(nodes.pair(3, 6), "muscle");

                
                
        
        // Start with a set of tgRigidInfo*. A tgStructureInfo will also make this available.
        std::vector<tgRigidInfo*> rigids;
        for(int i = 0; i < pairs.size(); i++) {
            rigids.push_back(new tgRodInfo(rodConfig, pairs[i]));
        }

        // Create compound rigids out of rigids that share nodes
        tgRigidAutoCompound autoCompound(rigids);
        autoCompound.execute(); // After this we have all rigids grouped (even if they're in their own group)

        // need to init the models in the world...
        std::vector<tgRigidInfo*>::iterator it;
        for(it = rigids.begin(); it != rigids.end(); it++) {
            (*it)->initRigidBody(world);
        }
        
        // not using this right now...
        //tgBuildSpec spec;
        //spec.addBuilder("rod", new tgRodInfo(rodConfig));  // @todo: should probably have tgRigidBuilder be abstract and use something like tgRodBuilder




        std::set<tgConnectorInfo*> muscles;
        for(int i = 0; i < musclePairs.size(); i++) {
            tgConnectorInfo* s = new tgBasicActuatorInfo(muscleConfig, musclePairs[i]);
            
            std::cout << "tgConnectorInfo: " << std::endl;
            std::cout << *s << std::endl;

            try {
                s->chooseRigids(rigids);
            } catch(std::exception e) {
                std::cout << "caught exception " << e.what() << std::endl;
                throw;
            }
            s->initConnector(world);
            addChild(s->createModel(world));
            //m_testString = s->createModel(world);
            //muscles.insert(s);
            //addChild(m_testString);
        }
        
        for(it = rigids.begin(); it != rigids.end(); it++) {
            (*it)->createModel(world);
        }

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
        
        //m_testString->onRender(r);
        
        tgModel::onVisit(r);
        //std::cout << "      ConnectorTestModel has special rendering requirements!" << std::endl;
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
