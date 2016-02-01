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
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include "helpers/FileHelpers.h"
#include <iostream>
#include <json/json.h>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
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

void TensegrityModel::setup(tgWorld& world) {
    // Parse Yaml File
    Yam root = YAML::LoadFile(yamlPath);

    // Create a structure that will hold the details of this model
    tgStructure structure;

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;

    addChildren(structure, spec, root["substructures"]);

    connectChildren(structure, root["link_groups"]);

    // Add nodes to the structure
    addNodes(structure, root["nodes"]);

    // Add pairs to the structure
    addPairs(structure, root["pair_groups"]);


    addBuilders(spec, root["builders"]);

    // Move the structure so it doesn't start in the ground
    structure.move(btVector3(0, 20, 0));

    // Create your structureInfo
    tgStructureInfo structureInfo(structure, spec);

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

void TensegrityModel::addChildren(tgStructure& structure, tgBuildSpec& spec, const Yam& substructures) {
    if (!substructures) return;
    for (YAML::const_iterator substructure = substructures.begin(); substructure!=substructures.end(); ++substructure) {
        tgStructure* childStructure = new tgStructure();
        string name = substructure->first.as<string>();
        Yam childRoot = YAML::LoadFile(substructure->second.as<string>());
        addNodes(*childStructure, childRoot["nodes"]);
        addPairs(*childStructure, childRoot["pair_groups"]);
        childStructure->addTags(name);
        structure.addChild(childStructure);
        addBuilders(spec, childRoot["builders"]);
    }
}

void TensegrityModel::connectChildren(tgStructure& structure, const Yam& linkGroups) {
    if (!linkGroups) return;
    for (YAML::const_iterator linkPairGroup = linkGroups.begin(); linkPairGroup!=linkGroups.end(); ++linkPairGroup) {

        string groupTags = linkPairGroup->first.as<string>();
        Yam groupLinks = linkPairGroup->second;

        string limbStructureNodePath = groupLinks.begin()->first.as<string>();
        string limbStructureName = limbStructureNodePath.substr(0, limbStructureNodePath.find("/"));
        tgStructure& limbStructure = structure.findFirstChild(limbStructureName);

        string bodyStructureNodePath = groupLinks.begin()->second[0].as<string>();
        string bodyStructureName = bodyStructureNodePath.substr(0, bodyStructureNodePath.find("/"));
        tgStructure& bodyStructure = structure.findFirstChild(bodyStructureName);

        connectChild(structure, limbStructure, bodyStructure, groupTags, groupLinks);
    }
}

void TensegrityModel::connectChild(tgStructure& structure, tgStructure& limbStructure, tgStructure& bodyStructure, string tags, const Yam& links) {
    if (!links) return;

    vector<tgNode*> limbLinkNodes;
    vector< pair<tgNode*, tgNode*> > bodyLinkPairs;
    vector<tgNode*> bodyLinkPairMidpoints;

    findLinkNodes(structure, limbLinkNodes, bodyLinkPairs, bodyLinkPairMidpoints, links);

    rotateAndTranslate(limbStructure, limbLinkNodes, bodyLinkPairMidpoints);

    for (unsigned int i = 0; i < limbLinkNodes.size(); i++) {

        //remove old connections
        bodyStructure.removePair(*(bodyLinkPairs[i].first), *(bodyLinkPairs[i].second));
        for (unsigned int j = 0; j < limbLinkNodes.size(); j++) {
            limbStructure.removePair(*limbLinkNodes[i], *limbLinkNodes[j]);
        }

        // make new connections
        structure.addPair(*(bodyLinkPairs[i].first), *limbLinkNodes[i], tags);
        structure.addPair(*limbLinkNodes[i], *(bodyLinkPairs[i].second), tags);

    }
}

void TensegrityModel::findLinkNodes(tgStructure& structure, vector<tgNode*>& limbLinkNodes,
    vector< pair<tgNode*, tgNode*> >& bodyLinkPairs, vector<tgNode*>& bodyLinkPairMidpoints, const Yam& links) {
    for (YAML::const_iterator link = links.begin(); link!=links.end(); ++link) {
        string limbNodePath = link->first.as<string>();
        Yam bodyPair = link->second;

        string limbStructurePath = getStructurePath(limbNodePath);
        string limbNodeName = getNodeName(limbNodePath);

        string bodyStructurePath = getStructurePath(bodyPair[0].as<string>());
        string pairNode1Name = getNodeName(bodyPair[0].as<string>());
        string pairNode2Name = getNodeName(bodyPair[1].as<string>());

        tgStructure& limbStructure = structure.findFirstChild(limbStructurePath);
        tgNodes& limbNodes = limbStructure.getNodes();
        limbLinkNodes.push_back(&(limbNodes.findFirst(limbNodeName)));

        tgStructure& bodyStructure = structure.findFirstChild(bodyStructurePath);
        tgNodes& bodyNodes = bodyStructure.getNodes();
        tgNode* bodyPairNode1 = &(bodyNodes.findFirst(pairNode1Name));
        tgNode* bodyPairNode2 = &(bodyNodes.findFirst(pairNode2Name));
        bodyLinkPairs.push_back(make_pair(bodyPairNode1, bodyPairNode2));

        tgNode* midpoint = new tgNode((*bodyPairNode1 + *bodyPairNode2) / 2);
        bodyLinkPairMidpoints.push_back(midpoint);
    }
}

string TensegrityModel::getStructurePath(string nodePath) const{
    return nodePath.substr(0, nodePath.rfind("/"));
}

string TensegrityModel::getNodeName(string nodePath) const {
    return nodePath.substr(nodePath.rfind("/") + 1);
}

void TensegrityModel::rotateAndTranslate(tgStructure& limbStructure, vector<tgNode*>& limbLinkNodes, vector<tgNode*>& bodyLinkPairMidpoints) {
    btVector3 limbLinkNodesCentroid = getCentroid(limbLinkNodes);
    btVector3 bodyLinkNodesCentroid = getCentroid(bodyLinkPairMidpoints);

    btVector3 limbPlaneCrossProduct = (*limbLinkNodes[1] - *limbLinkNodes[0]).cross(*limbLinkNodes[2] - *limbLinkNodes[0]);
    btVector3 limbPlaneNormal = limbPlaneCrossProduct.normalize();

    btVector3 bodyPlaneCrossProduct = (*bodyLinkPairMidpoints[1] - *bodyLinkPairMidpoints[0]).cross(*bodyLinkPairMidpoints[2] - *bodyLinkPairMidpoints[0]);
    btVector3 bodyPlaneNormal = bodyPlaneCrossProduct.normalize();

    // rotate and translate
    limbStructure.addRotation(limbLinkNodesCentroid, limbPlaneNormal, bodyPlaneNormal);
    limbStructure.move(bodyLinkNodesCentroid - limbLinkNodesCentroid);
    limbStructure.addRotation(bodyLinkNodesCentroid, *limbLinkNodes[0] - bodyLinkNodesCentroid, *bodyLinkPairMidpoints[0] - bodyLinkNodesCentroid);
}

btVector3 TensegrityModel::getCentroid(const vector<tgNode*>& points) const {
    int numPoints = points.size();
    btVector3 centroid(0,0,0);
    for (int i = 0; i < numPoints; i++) {
        centroid += *points[i];
    }
    centroid /= numPoints;
    return centroid;
}

void TensegrityModel::addNodes(tgStructure& structure, const Yam& nodes)
{
    if (!nodes) return;
    for (YAML::const_iterator node = nodes.begin(); node!=nodes.end(); ++node) {
        string name = node->first.as<string>();
        Yam xyz = node->second;
        double x = xyz[0].as<double>();
        double y = xyz[1].as<double>();
        double z = xyz[2].as<double>();
        structure.addNode(x, y, z, name);
    }
}

void TensegrityModel::addPairs(tgStructure& structure, const Yam& pair_groups)
{
    if (!pair_groups) return;
    tgNodes nodes = structure.getNodes();
    for (YAML::const_iterator pair_group = pair_groups.begin(); pair_group!=pair_groups.end(); ++pair_group) {
        string tags = pair_group->first.as<string>();
        Yam pairs = pair_group->second;
        for (unsigned int i = 0; i < pairs.size(); i++) {
            string node1Name = pairs[i][0].as<string>();
            tgNode node1 = nodes.findFirst(node1Name);
            string node2Name = pairs[i][1].as<string>();
            tgNode node2 = nodes.findFirst(node2Name);
            tags += " " + node1Name + " " + node2Name;
            structure.addPair(node1, node2, tags);
        }
    }
}

void TensegrityModel::addBuilders(tgBuildSpec& spec, const Yam& builders) {

    map<string, double> rodParameters;
    map<string, double> stringParameters;

    // default rod params
    rodParameters["radius"] = 0.5;
    rodParameters["density"] = 1.0;
    rodParameters["friction"] = 1.0;
    rodParameters["rollFriction"] = 0.0;
    rodParameters["restitution"] = 0.2;

    // default muscle params
    stringParameters["stiffness"] = 1000.0;
    stringParameters["damping"] = 10.0;
    stringParameters["pretension"] = 0.0;
    stringParameters["hist"] = 0;
    stringParameters["maxTens"] = 1000.0;
    stringParameters["targetVelocity"] = 100.0;
    stringParameters["minActualLength"] = 0.1;
    stringParameters["minRestLength"] = 0.1;
    stringParameters["rotation"] = 0;

    if (!builders || builders.size() == 0) {
        const tgRod::Config* rodConfig = new tgRod::Config(rodParameters["radius"], rodParameters["density"],
            rodParameters["friction"], rodParameters["rollFriction"], rodParameters["restitution"]);
        spec.addBuilder("rods", new tgRodInfo(*rodConfig));
        const tgBasicActuator::Config* muscleConfig = new tgBasicActuator::Config(stringParameters["stiffness"], stringParameters["damping"],
            stringParameters["pretension"], stringParameters["hist"], stringParameters["maxTens"], stringParameters["targetVelocity"],
            stringParameters["minActualLength"], stringParameters["minRestLength"], stringParameters["rotation"]);
        spec.addBuilder("strings", new tgBasicActuatorInfo(*muscleConfig));
    }

    for (YAML::const_iterator builder = builders.begin(); builder!=builders.end(); ++builder) {
        string tag_match = builder->first.as<string>();
        string builderClass = builder->second["class"].as<string>();
        Yam parameters = builder->second["parameters"];
        if (builderClass == "tgRodInfo") {
            for (YAML::const_iterator parameter = parameters.begin(); parameter!=parameters.end(); ++parameter) {
                rodParameters[parameter->first.as<string>()] = parameter->second.as<double>();
            }
            const tgRod::Config* rodConfig = new tgRod::Config(rodParameters["radius"], rodParameters["density"],
                rodParameters["friction"], rodParameters["rollFriction"], rodParameters["restitution"]);
            spec.addBuilder(tag_match, new tgRodInfo(*rodConfig));
        }
        else {
            for (YAML::const_iterator parameter = parameters.begin(); parameter!=parameters.end(); ++parameter) {
                stringParameters[parameter->first.as<string>()] = parameter->second.as<double>();
            }
            if (builderClass == "tgBasicActuatorInfo") {
                const tgBasicActuator::Config* muscleConfig = new tgBasicActuator::Config(stringParameters["stiffness"], stringParameters["damping"],
                    stringParameters["pretension"], stringParameters["hist"], stringParameters["maxTens"], stringParameters["targetVelocity"],
                    stringParameters["minActualLength"], stringParameters["minRestLength"], stringParameters["rotation"]);
                spec.addBuilder(tag_match, new tgBasicActuatorInfo(*muscleConfig));
            }
            else {
                throw invalid_argument(builderClass + " builder class is not supported.");
            }
        }
    }
}

void TensegrityModel::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw invalid_argument("dt is not positive");
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

const vector<tgSpringCableActuator*>& TensegrityModel::getAllActuators() const
{
    return allActuators;
}

void TensegrityModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
