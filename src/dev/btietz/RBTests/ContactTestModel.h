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

#ifndef CONTACT_TEST_MODEL_H
#define CONTACT_TEST_MODEL_H

#include "core/tgModel.h" 
#include "core/tgLinearString.h"

#include "core/tgSubject.h"

#include "btBulletDynamicsCommon.h"
#include <iostream>
#include <set>

#include "tgcreator/tgRodInfo.h"
#include "core/tgLinearString.h"
#include "dev/btietz/tgRBString.h"
#include "tgcreator/tgLinearStringInfo.h"
#include "dev/btietz/tgRBStringInfo.h"
#include "tgcreator/tgRigidAutoCompound.h"
#include "tgcreator/tgBuildSpec.h"

#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgUtil.h"
#include "core/tgString.h"

//#include <tgmath.h>
/**
 * Depricated as of version 1.1.0
 */
class ContactTestModel : public tgSubject<ContactTestModel>, public tgModel
{
public: 

    ContactTestModel() : tgModel() 
    {
    }

    virtual ~ContactTestModel()
    {}
    
    /** @todo Get rid of this. */
    virtual void setup(tgWorld& world)
    {
        
        // Config for rods that are a part of RB Strings
        const double radius  = 0.15;
        const double density = 4;  // Note: This needs to be high enough or things fly apart...
        const tgRod::Config rodConfig(radius, density);
        
        const double stiffness = 10000.0;
        const double damping = .01*stiffness;
        const int segments = 16;
        const tgLinearString::Config stringConfig(stiffness, damping);
        const tgRBString::Config rbConfig(segments, rodConfig, stringConfig, 8.0);
        
        // Config for the other rods
        const double bigRadius = 1.0;
        const double massLess = 0;
        const double massive = 1.0;
        
        const tgRod::Config masslessRod(bigRadius, massLess);
        const tgRod::Config massiveRod(bigRadius, massive);
        
        double length = 10.0;
        double height = 20.0;
                
        tgStructure s;

        s.addNode(0,  height, 0);                   // 0
        s.addNode(length,   height, 0);            // 1
        s.addNode(0,  height + 10.0, 0);           // 2
        s.addNode(length,   height + 10.0, 0);     // 3

        // Rods
        s.addPair( 0,  1, "massive rod");
        s.addPair( 2,  3, "massless rod");

                
        // Muscles
        s.addPair(0, 2,  "muscle");
        s.addPair(1, 3,  "muscle");

        //s.move(btVector3(0, 10, 0));
        
        // Create the build spec that uses tags to turn the structure into a real model
        tgBuildSpec spec;
        spec.addBuilder("massive", new tgRodInfo(massiveRod));
        spec.addBuilder("massless", new tgRodInfo(masslessRod));
        spec.addBuilder("muscle", new tgRBStringInfo(rbConfig));
        
        // Create your structureInfo
        tgStructureInfo structureInfo(s, spec);

        // Use the structureInfo to build ourselves
        structureInfo.buildInto(*this, world);

        // We could now use tgCast::filter or similar to pull out the models (e.g. muscles) that we want to control. 
        allMuscles = tgCast::filter<tgModel, tgRBString> (getDescendants());

        // Debug printing
        std::cout << "StructureInfo:" << std::endl;
        std::cout << structureInfo << std::endl;
        
        std::cout << "Model: " << std::endl;
        std::cout << *this << std::endl;
        
        // Actually setup the children
        tgModel::setup(world);
        
        std::cout << "Finished Setup!" << std::endl;

    }
    
    // @todo: Is there a way that we can get rid of the need to override this function in subclasses of tgModel? 
    // comment_BRT: only if we can somehow make tgModel a template class,
    // we need to know what class we're notifying
    virtual void step(double dt)
    {
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        
        tgModel::step(dt);  // Step any children
    
    }

    void teardown()
    {
        tgModel::teardown();
    }

    std::vector <tgRBString*>& getAllMuscles ()
    {
        return allMuscles;
    }
    
private:
    std::vector<tgRBString*> allMuscles;
};

#endif
