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

string TensegrityModel::yamlPath;

TensegrityModel::TensegrityModel(string j) :
tgModel() 
{
    yamlPath = j;
}

TensegrityModel::~TensegrityModel()
{
}

void TensegrityModel::addNodes(tgStructure& s, YAML::Node root)
{
    YAML::Node nodes = root["structure"]["nodes"];
    for (unsigned int i = 0; i < nodes.size(); i++) {
        YAML::Node xyz = nodes[i]["node"]["xyz"];
        double x = xyz[0].as<double>();
        double y = xyz[1].as<double>();
        double z = xyz[2].as<double>();
        s.addNode(x, y, z);
    }
}

void TensegrityModel::addPairs(tgStructure& s, YAML::Node root)
{
    YAML::Node pairs = root["structure"]["pairs"];
    for (unsigned int i = 0; i < pairs.size(); i++) {
        YAML::Node pair = pairs[i]["pair"];
        int n1 = pair["start"].as<int>();
        int n2 = pair["end"].as<int>();
        string tags = pair["tags"].as<string>();
        s.addPair(n1 - 1, n2 - 1, tags);
    } 
}

void TensegrityModel::setup(tgWorld& world)
{
    // Parse Yaml File
    YAML::Node root = YAML::LoadFile(yamlPath);

    // YAML::Node rodParams = root["parameters"]["rods"];
    // YAML::Node muscleParams = root["parameters"]["muscles"];

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

    // // set rod params from JSON
    // if (rodParams.isMember("radius"))
    //     radius = rodParams["radius"].asDouble();
    // if (rodParams.isMember("density"))
    //     density = rodParams["density"].asDouble();
    // if (rodParams.isMember("friction"))
    //     friction = rodParams["friction"].asDouble();
    // if (rodParams.isMember("rollFriction"))
    //     rollFriction = rodParams["rollFriction"].asDouble();
    // if (rodParams.isMember("restitution"))
    //     restitution = rodParams["restitution"].asDouble();

    // // set muscle params from JSON
    // if (muscleParams.isMember("stiffness"))
    //    stiffness = muscleParams["stiffness"].asDouble();
    // if (muscleParams.isMember("damping"))
    //     damping = muscleParams["damping"].asDouble();
    // if (muscleParams.isMember("pretension"))
    //     pretension = muscleParams["pretension"].asDouble();
    // if (muscleParams.isMember("hist"))
    //     hist = muscleParams["hist"].asDouble();
    // if (muscleParams.isMember("maxTens"))
    //     maxTens = muscleParams["maxTens"].asDouble();
    // if (muscleParams.isMember("targetVelocity"))
    //     targetVelocity = muscleParams["targetVelocity"].asDouble();
    // if (muscleParams.isMember("minActualLength"))
    //     minActualLength = muscleParams["minActualLength"].asDouble();
    // if (muscleParams.isMember("minRestLength"))
    //     minRestLength = muscleParams["minRestLength"].asDouble();
    // if (muscleParams.isMember("rotation"))
    //     rotation = muscleParams["rotation"].asDouble();

     // Define the configurations of the rods and strings
     const tgRod::Config rodConfig(radius, density, friction, rollFriction, restitution);
     const tgSpringCableActuator::Config muscleConfig(stiffness, damping, pretension, hist, maxTens, targetVelocity, minActualLength, minRestLength, rotation);
    
    // Create a structure that will hold the details of this model
    tgStructure s;
    
    // Add nodes to the structure
    addNodes(s, root);
    
    // Add pairs to the structure
    addPairs(s, root);
        
    // Move the structure so it doesn't start in the ground
    s.move(btVector3(0, 10, 0));
    
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    
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
