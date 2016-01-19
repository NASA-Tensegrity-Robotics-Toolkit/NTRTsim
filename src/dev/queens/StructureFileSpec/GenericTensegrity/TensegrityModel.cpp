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
 * @file TensegrityModel.cpp
 * @brief Contains the definition of the members of the class TensegrityModel.
 * $Id$
 */

// This module
#include "TensegrityModel.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>
#include "helpers/FileHelpers.h"
#include <json/json.h>
#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>

using namespace std;

typedef YAML::Node Yam; // to avoid confusion with structure nodes

string TensegrityModel::yamlPath;

TensegrityModel::TensegrityModel(string j) :
tgModel() 
{
    yamlPath = j;
}

TensegrityModel::~TensegrityModel()
{
}

void TensegrityModel::addNodes(tgStructure& s, const Yam& root)
{
    Yam nodes = root["structure"]["nodes"];
    for (YAML::const_iterator node = nodes.begin(); node!=nodes.end(); ++node) {
        string name = node->first.as<string>();
        Yam xyz = node->second;
        double x = xyz[0].as<double>();
        double y = xyz[1].as<double>();
        double z = xyz[2].as<double>();
        s.addNode(x, y, z, name);
    }
}

void TensegrityModel::addPairs(tgStructure& s, const Yam& root)
{
    tgNodes nodes = s.getNodes();
    Yam pair_groups = root["structure"]["pair_groups"];
    for (YAML::const_iterator pair_group = pair_groups.begin(); pair_group!=pair_groups.end(); ++pair_group) {
        string tags = pair_group->first.as<string>();
        Yam pairs = pair_group->second;
        for (unsigned int i = 0; i < pairs.size(); i++) {
            string node1Name = pairs[i][0].as<string>();
            int node1Index = nodes.findFirstIndex(node1Name);
            string node2Name = pairs[i][1].as<string>();
            int node2Index = nodes.findFirstIndex(node2Name);
            s.addPair(node1Index, node2Index, tags);
        }
    } 
}

void TensegrityModel::setup(tgWorld& world)
{

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;

    // Parse Yaml File
    Yam root = YAML::LoadFile(yamlPath);

    // default rod params
    double radius = 0.5;
    double density = 1.0;
    double friction = 1.0;
    double rollFriction = 0.0;
    double restitution = 0.2;

    // default muscle params
    double stiffness = 1000.0;
    double damping = 10.0;
    double pretension = 0.0;
    double hist = false;
    double maxTens = 1000.0;
    double targetVelocity = 100.0;
    double minActualLength = 0.1;
    double minRestLength = 0.1;
    double rotation = 0;

    // Define the configurations of the rods and strings
    const tgRod::Config rodConfig(radius, density, friction, rollFriction, restitution);
    spec.addBuilder("rods", new tgRodInfo(rodConfig));
    const tgBasicActuator::Config muscleConfig(stiffness, damping, pretension, hist, maxTens, targetVelocity, minActualLength, minRestLength, rotation);
    spec.addBuilder("muscles", new tgBasicActuatorInfo(muscleConfig));

    // Create a structure that will hold the details of this model
    tgStructure s;
    
    // Add nodes to the structure
    addNodes(s, root);
    
    // Add pairs to the structure
    addPairs(s, root);
        
    // Move the structure so it doesn't start in the ground
    s.move(btVector3(0, 10, 0));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allActuators = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());
    
    // Notify controllers that setup has finished.
    notifySetup();
    
    // Actually setup the children
    tgModel::setup(world);
}

void TensegrityModel::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        tgModel::step(dt);  // Step any children
    }
}

void TensegrityModel::onVisit(tgModelVisitor& r)
{
    // Example: m_rod->getRigidBody()->dosomething()...
    tgModel::onVisit(r);
}

const std::vector<tgSpringCableActuator*>& TensegrityModel::getAllActuators() const
{
    return allActuators;
}
    
void TensegrityModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
