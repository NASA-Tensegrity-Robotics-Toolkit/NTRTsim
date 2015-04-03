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

/**
 * @file tgCraterShallow.cpp
 * @brief Contains the implementation of class tgCraterShallow.
 * This tgCraterShallow is four shallow boxes that encircle the origin.
 * $Id$
 */

// This module
#include "tgCraterShallow.h"
// This library
#include "core/tgBox.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBoxInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgNode.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>
#include <vector>

namespace
{
    // similarly, frictional parameters are for the tgRod objects.
    const struct Config
    {
        double width;
        double height;
        double density;
        double friction;
        double rollFriction;
        double restitution;
    } c =
    {
        10.0, // width (dm?)
        10.0, // height (dm?)
        0.0,  // density (kg / length^3)
        1.0,  // friction (unitless)
        0.01, // rollFriction (unitless)
        0.2,  // restitution (?)
    };
} // namespace

tgCraterShallow::tgCraterShallow() : tgModel() {
    origin = btVector3(0,0,0);
}

tgCraterShallow::tgCraterShallow(btVector3 center) : tgModel() {
    origin = btVector3(center.getX(), center.getY(), center.getZ());
}

tgCraterShallow::~tgCraterShallow() {}
                     
void tgCraterShallow::setup(tgWorld& world) {
    const tgBox::Config boxConfig(c.width, c.height, c.density, c.friction, c.rollFriction, c.restitution);

    // Start creating the structure
    tgStructure s;
    addNodes(s);

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("box", new tgBoxInfo(boxConfig));

    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // call the onSetup methods of all observed things e.g. controllers
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);
}

void tgCraterShallow::step(double dt) {
    // Precondition
    if (dt <= 0.0) {
        throw std::invalid_argument("dt is not positive");
    }
    else { //Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        tgModel::step(dt); // Step any children
    }
}

void tgCraterShallow::onVisit(tgModelVisitor& r) {
    tgModel::onVisit(r);
}

void tgCraterShallow::teardown() {
    notifyTeardown();
    tgModel::teardown();
} 

// Nodes: center points of opposing faces of rectangles
void tgCraterShallow::addNodes(tgStructure& s) {
    
#if (0)    
    const int nBoxes = 4; 
#endif // Supress compiler warning unused variable
    // Accumulating rotation on boxes
    btVector3 rotationPoint = origin;
    btVector3 rotationAxis = btVector3(0, 1, 0);  // y-axis
    double rotationAngle = M_PI/2;

    addBoxNodes();
    addBoxNodes();
    addBoxNodes();
    addBoxNodes();
    
    for(std::size_t i=0;i<nodes.size();i+=2) {
        s.addNode(nodes[i]);
        s.addNode(nodes[i+1]);
        s.addRotation(rotationPoint, rotationAxis, rotationAngle);
        s.addPair(i, i+1, "box");
    }
    s.move(btVector3(0, -5, 0)); // Sink boxes into the ground
}

void tgCraterShallow::addBoxNodes() {
    tgNode node;
    const double shift = 20; // Arbitrary
    const double vshift = 2; 
    const double node_h = c.height/2 + vshift;
    const double node_w = c.width/2; 
    
    double x1 = -shift-node_w;
    double x2 =  shift+node_w;
    double y1 = -node_h;
    double y2 =  node_h;
    double z1 = -shift-node_w;
    double z2 =  shift+node_w;

    btVector3 rotationPoint = btVector3((x2-x1)/2, (y2-y1)/2, (z2-z1)/2); //Halfway between nodes
    btVector3 rotationAxis = btVector3(0, 1, 0);  // y-axis
    double rotationAngle = M_PI/4;

    node = tgNode(x1, y1, z1, "node");
    node.addRotation(rotationPoint, rotationAxis, rotationAngle);
    nodes.push_back(node);

    node = tgNode(x2, y2, z2, "node");
    node.addRotation(rotationPoint, rotationAxis, rotationAngle);
    nodes.push_back(node);
     
}

