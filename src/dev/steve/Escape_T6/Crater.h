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
 * @file Crater.h
 * @brief Contains the definition of class Crater.
 * $Id$
 */

// This library
#include "core/tgModel.h"
#include "core/tgSubject.h"
#include "core/tgRod.h"
// The C++ Standard Library
#include <vector>

// Forward declarations
class tgLinearString;
class tgModelVisitor;
class tgStructure;
class tgWorld;
class tgNode;

/**
 * Class that creates box "model" for the sake of 
 * levelling off the existing ground
 */
class Crater : public tgSubject<Crater>, public tgModel
{
    public: 

        Crater();
        Crater(btVector3 origin);

        /**
         * Destructor. Deletes controllers, if any were added during setup.
         * Teardown handles everything else.
         */
        virtual ~Crater();

        /**
         * Create the model.
         * @param[in] world - the world we're building into
         */
        virtual void setup(tgWorld& world);

        /**
         * Undoes setup. Deletes child models. Called automatically on
         * reset and end of simulation. Notifies controllers of teardown
         */
        void teardown();

        /**
         * Step the model, its children. Notifies controllers of step.
         * @param[in] dt, the timestep. Must be positive.
         */
        virtual void step(double dt);

        /**
         * Receives a tgModelVisitor and dispatches itself into the
         * visitor's "render" function. This model will go to the default
         * tgModel function, which does nothing.
         * @param[in] r - a tgModelVisitor which will pass this model back
         * to itself 
         */
        virtual void onVisit(tgModelVisitor& r);

    private:

        /**
         * A function called during setup that determines the positions of
         * the nodes based on construction parameters. Rewrite this function
         * for your own models
         * @param[in] tetra: A tgStructure that we're building into
         */
        void addNodes(tgStructure& s);

        /**
         * Moves all the rods (that are actually all the rigid bodies) according to the arguments.
         * First rotates the structure around 3 axises given 3 angles.
         * Moves the structure to the target point.
         * Sets all the bars speed to the given speed vector.
         * (muscles and markers are moved automatically since they are attached).
         */
        void moveModel(btVector3 targetPositionVector,btVector3 rotationVector,btVector3 speedVector);

        void addBoxNodes();

        std::vector<std::vector<std::vector<int> > > nodeNumberingSchema;

        std::vector <tgNode> nodes;
        btVector3 origin;
};

