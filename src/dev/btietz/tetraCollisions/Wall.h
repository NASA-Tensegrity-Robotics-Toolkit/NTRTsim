/*
 * Copyright © 2014, United States Government, as represented by the
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

#ifndef TETRA_COLLISIONS_WALL
#define TETRA_COLLISIONS_WALL

/**
 * @file Wall.h
 * @brief Contains the definition of class Wall.
 * Specifically, a crater is defined as a series of boxes which
 * form a barrier. It is designed to test escape algorithms for tensegrities
 * $Id$
 */

// This library
#include "core/tgModel.h"
#include "core/tgSubject.h"
// The Bullet Physics Library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <vector>

// Forward declarations
class tgModelVisitor;
class tgStructure;
class tgWorld;
class tgNode;

/**
 * Class that creates box "models" that act as crater walls
 */
class Wall : public tgSubject<Wall>, public tgModel
{
    public: 

        /**
         * Default constructor. Sets center point of crater to (0,0,0)
         */
        Wall();

        /**
         * Origin constructor. Sets center point to input param 'origin'.
         * @param[in] origin - the center point of the Wall object
         */
        Wall(btVector3 origin);

        /**
         * Destructor. Deletes controllers, if any were added during setup.
         * Teardown handles everything else.
         */
        virtual ~Wall();

        /**
         * Create the model.
         * @param[in] world - the world we're building into
         */
        virtual void setup(tgWorld& world);

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
 
        /**
         * Undoes setup. Deletes child models. Called automatically on
         * reset and end of simulation. Notifies controllers of teardown
         */
        void teardown();  

    private:

        /**
         * A function called during setup that determines the positions of
         * the nodes (center points of opposing box faces) 
         * based on construction parameters.
         * @param[in] s: the tgStructure that we're building into
         */
        void addNodes(tgStructure& s);

        /**
         * Determines the box nodes (center points of opposing box faces
         * Adds nodes to 'nodes' vector
         */
        void addBoxNodes();

        std::vector <tgNode> nodes;
        btVector3 origin;
};

#endif // TETRA_COLLISIONS_WALL
