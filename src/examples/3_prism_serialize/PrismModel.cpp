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
 * @file PrismModel.cpp
 * @brief Contains a modified version of 3_prism to use JSON in place of a Config struct for structure configuration.
 * $Id$
 */

// This module
#include "PrismModel.h"
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
#include <string>
#include "helpers/FileHelpers.h"
#include <json/json.h>

using namespace std;

/**
 * Anonomous namespace so we don't have to declare the config in
 * the header.
 */
namespace
{
    /**
     * Configuration parameters so they're easily accessable.
     * All parameters must be positive.
     */
    const struct Config
    {
        double density;
        double radius;
        double stiffness;
        double damping;
        double triangle_length;
        double triangle_height;
        double prism_height;
        double pretension;     
    } c =
    {
        0.2,     // density (mass / length^3)
        0.31,     // radius (length)
        1000.0,   // stiffness (mass / sec^2)
        10.0,     // damping (mass / sec)
        10.0,     // triangle_length (length)
        10.0,     // triangle_height (length)
        20.0,     // prism_height (length)
        500.0      // Pretension (mass * length / sec^2)
    };
} // namespace

PrismModel::PrismModel() :
    tgModel() 
{
}

PrismModel::~PrismModel()
{

}

void PrismModel::addNodes(tgStructure& tetra,
        double edge,
        double width,
        double height)
{
    // bottom right
    tetra.addNode(-edge / 2.0, 0, 0); // 1
    // bottom left
    tetra.addNode( edge / 2.0, 0, 0); // 2
    // bottom front
    tetra.addNode(0, 0, width); // 3
    // top right
    tetra.addNode(-edge / 2.0, height, 0); // 4
    // top left
    tetra.addNode( edge / 2.0, height, 0); // 5
    // top front
    tetra.addNode(0, height, width); // 6
}

void PrismModel::addRods(tgStructure& s)
{
    s.addPair( 0,  4, "rod");
    s.addPair( 1,  5, "rod");
    s.addPair( 2,  3, "rod");
}

void PrismModel::addActuators(tgStructure& s)
{
    // Bottom Triangle
    s.addPair(0, 1,  "actuator");
    s.addPair(1, 2,  "actuator");
    s.addPair(2, 0,  "actuator");

    // Top
    s.addPair(3, 4, "actuator");
    s.addPair(4, 5,  "actuator");
    s.addPair(5, 3,  "actuator");

    //Edges
    s.addPair(0, 3, "actuator");
    s.addPair(1, 4,  "actuator");
    s.addPair(2, 5,  "actuator");
}

void PrismModel::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    const tgRod::Config rodConfig(c.radius, c.density);
    const tgBasicActuator::Config actuatorConfig(c.stiffness, c.damping, c.pretension);

    // Create a structure that will hold the details of this model
    tgStructure s;

    //BEGIN DESERIALIZING

    Json::Value root; // will contains the root value after parsing.
    Json::Reader reader;

    std::string configPath = FileHelpers::getResourcePath("3_prism_serialize/config.json");
    bool parsingSuccessful = reader.parse( FileHelpers::getFileString(configPath), root );
    if ( !parsingSuccessful )
    {
        // report to the user the failure and their locations in the document.
        std::cout << "Failed to parse configuration\n"
            << reader.getFormattedErrorMessages();
        return;
    }
    // Get the value of the member of root named 'encoding', return 'UTF-8' if there is no
    // such member.
    double triangle_height = root.get("triangle_height", "UTF-8").asDouble();
    double triangle_length = root.get("triangle_length", "UTF-8").asDouble();
    double prism_height = root.get("prism_height", "UTF-8").asDouble();
    //END SERIALIZING

    // Add nodes to the structure
    addNodes(s, triangle_length, triangle_height, prism_height);

    // Add rods to the structure
    addRods(s);

    // Add actuators to the structure
    addActuators(s);

    // Move the structure so it doesn't start in the ground
    s.move(btVector3(0, 10, 0));

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("actuator", new tgBasicActuatorInfo(actuatorConfig));

    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. actuators) that we want to control. 
    allActuators = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());

    // Notify controllers that setup has finished.
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);
}

void PrismModel::step(double dt)
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

void PrismModel::onVisit(tgModelVisitor& r)
{
    // Example: m_rod->getRigidBody()->dosomething()...
    tgModel::onVisit(r);
}

const std::vector<tgSpringCableActuator*>& PrismModel::getAllActuators() const
{
    return allActuators;
}

void PrismModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
