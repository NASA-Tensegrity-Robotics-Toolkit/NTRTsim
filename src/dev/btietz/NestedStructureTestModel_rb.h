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
#include "core/tgLinearString.h"
#include "dev/btietz/tgRBString.h"

#include "core/tgSubject.h"

#include "btBulletDynamicsCommon.h"
#include <iostream>
#include <set>
#include <map>

#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgLinearStringInfo.h"
#include "tgcreator/tgRBStringInfo.h"
#include "tgcreator/tgRigidAutoCompound.h"
#include "tgcreator/tgBuildSpec.h"

#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "core/tgUtil.h"
#include "core/tgString.h"

#include "core/tgCast.h"

class NestedStructureTestModel: public tgSubject<NestedStructureTestModel>, public tgModel
{
public: 

    NestedStructureTestModel(int segments) : 
        m_segments(segments),
        tgModel() 
    {
    }

    virtual ~NestedStructureTestModel()
    {}
    
    virtual void setup(tgWorld* world)
    {
        // This is basically a manual setup of a model. There are things that do this for us (@todo: reference the things that do this for us)

        // Rod and Muscle configuration
        
        const double density = 4.2/300.0;  // Note: This needs to be high enough or things fly apart...
        const double radius  = 0.5;
        const tgRod::Config rodConfig(radius, density);
        
        tgLinearString::Config muscleConfig(1000, 10);
        
        // Config for rods that are a part of RB Strings
        const double strRadius  = 0.2;
        const double strDensity = 2;  // Note: This needs to be high enough or things fly apart...
        const tgRod::Config strRodConfig(strRadius, strDensity);
        
        const double stiffness = 1000.0;
        const double damping = .01*stiffness;
        const int segments = 10;
        const tgLinearString::Config stringConfig(stiffness, damping);
        const tgRBStringInfo::Config rbConfig(segments, strRodConfig, stringConfig, 3.0);
        
        // Calculations for the tetrahedra model
        double edge = 30.0;
        double height = tgUtil::round(std::sqrt(3.0)/2 * edge);
        
        std::cout << "edge: " << edge << "; height: " << height << std::endl;
        
        // Create the tetrahedra
        tgStructure tetra;

        tetra.addNode(-edge/2,0,0);  // right
        tetra.addNode(edge/2,0,0);   // left
        tetra.addNode(0,height,0);   // top
        tetra.addNode(0,height/2, tgUtil::round(std::sqrt(3.0)/2 * height)); // front
        
        tetra.addPair(0,1, "back bottom rod");
        tetra.addPair(0,2, "back right rod");
        tetra.addPair(0,3, "front right rod");
        tetra.addPair(1,2, "back left rod");
        tetra.addPair(1,3, "front left rod");
        tetra.addPair(2,3, "front top rod");

        // Move the first one so we can create a longer snake. Or you could move the snake at the end, up to you. 
        tetra.move(btVector3(0.0,2.0,100.0));

        // Create our snake segments
        tgStructure snake;
        btVector3 offset(0,0,-edge * 1.15); // @todo: there seems to be an issue with Muscle2P connections if the front of a tetra is inside the next one.
        for(int i = 0; i < m_segments; i++) {
            // @todo: the snake is a temporary variable -- will its destructor be called? If not, where do we delete its children?
            tgStructure* t = new tgStructure(tetra);
            t->addTags(tgString("segment", i + 1));
            t->move((i + 1)*offset);
            snake.addChild(t); // Add a child to the snake
        }
        //conditionally compile for debugging 
        #if (1)
        // Add muscles that connect the segments
        std::vector<tgStructure*> children = snake.getChildren();
        for(int i = 1; i < children.size(); i++) {
            tgNodes n0 = children[i-1]->getNodes();
            tgNodes n1 = children[i]->getNodes();

            snake.addPair(n0[0], n1[0], "outer right muscle");
            snake.addPair(n0[1], n1[1], "outer left muscle");
            snake.addPair(n0[2], n1[2], "outer top muscle");

            snake.addPair(n0[0], n1[3], "inner right muscle");
            snake.addPair(n0[1], n1[3], "inner left muscle");
            snake.addPair(n0[2], n1[3], "inner top muscle");

        }
        #endif
        // Create the build spec that uses tags to turn the structure into a real model
        tgBuildSpec spec;
        spec.addBuilder("rod", new tgRodInfo(rodConfig));
        
        #if (1)
        spec.addBuilder("muscle", new tgRBStringInfo(rbConfig));
        #endif
        
        // Create your structureInfo
        tgStructureInfo structureInfo(&snake, &spec);

        // Use the structureInfo to build ourselves
        structureInfo.buildInto(this, world);

        // We could now use tgCast::filter or similar to pull out the models (e.g. muscles) that we want to control. 
        
        allMuscles = tgCast::filter<tgModel, tgLinearString> (getDescendants());
        
        // Note that tags don't need to match exactly, we could create
        // supersets if we wanted to
        muscleMap["inner left"] = this->find<tgLinearString>("inner left muscle");
        muscleMap["inner right"] = this->find<tgLinearString>("inner right muscle");
        muscleMap["inner top"] = this->find<tgLinearString>("inner top muscle");
        muscleMap["outer left"] = this->find<tgLinearString>("outer left muscle");
        muscleMap["outer right"] = this->find<tgLinearString>("outer right muscle");
        muscleMap["outer top"] = this->find<tgLinearString>("outer top muscle");

        // Debug printing
        std::cout << "StructureInfo:" << std::endl;
        std::cout << structureInfo << std::endl;
        
        std::cout << "Model: " << std::endl;
        std::cout << *this << std::endl;

        
        // Showing the find function
        std::vector<tgLinearString*> outerMuscles = find<tgLinearString>("outer");
        for(int i = 0; i < outerMuscles.size(); i++) {
            std::cout << "Outer muscle: " << *(outerMuscles[i]) << std::endl;
        }


    }
    
    virtual void step(double dt)
    {
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        
        tgModel::step(dt);  // Step any children
    
    }

    //Does this make myModel an observer as well??
    void changeMuscle (double length, double dt)
    {
        for( int i = 0; i < allMuscles.size(); i++){
            allMuscles[i]->setRestLength(length, dt);
        }
    }
    const std::vector<tgLinearString*> getMuscles (std::string key)
    {
        return muscleMap[key];
    }
    
    const int getSegments()
    {
        return m_segments;
    }
    
private:
    std::vector<tgLinearString*> allMuscles;
    std::map<std::string, std::vector<tgLinearString*> > muscleMap;
    
    const int m_segments;
};

#endif
