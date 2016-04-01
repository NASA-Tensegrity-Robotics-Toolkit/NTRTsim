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
 * @author Simon Kotwicz & Jonah Eisen
 * $Id$
 */

#include "TensegrityModel.h"
// C++ Standard Library
#include <iostream>
#include <stdexcept>
// NTRT Core and tgCreator Libraries
#include "core/tgBasicActuator.h"
#include "core/tgKinematicActuator.h"
#include "core/tgRod.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgBasicContactCableInfo.h"
#include "tgcreator/tgKinematicActuatorInfo.h"
#include "tgcreator/tgKinematicContactCableInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructureInfo.h"

TensegrityModel::TensegrityModel(const std::string& structurePath) : tgModel() {
    topLvlStructurePath = structurePath;
}

TensegrityModel::~TensegrityModel() {}

void TensegrityModel::setup(tgWorld& world) {
    // create the build spec that uses tags to turn the structure into a model
    tgBuildSpec spec;

    // add default rod and string builders that match the tags rods & strings
    // (these will be overwritten if a different builder is specified for those tags)
    Yam emptyYam = Yam();
    addRodBuilder("tgRodInfo", "rod", emptyYam, spec);
    addBasicActuatorBuilder("tgBasicActuatorInfo", "string", emptyYam, spec);

    tgStructure structure;
    buildStructure(structure, topLvlStructurePath, spec);

    tgStructureInfo structureInfo(structure, spec);
    structureInfo.buildInto(*this, world);

    // use tgCast::filterto pull out the muscles that we want to control
    allActuators = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());

    // notify controllers that setup has finished
    notifySetup();

    // actually setup the children
    tgModel::setup(world);
}

void TensegrityModel::addChildren(tgStructure& structure, const std::string& structurePath, tgBuildSpec& spec, const Yam& children) {
    if (!children) return;

    // add all the children first
    for (YAML::const_iterator child = children.begin(); child != children.end(); ++child) {
        Yam childAttributes = child->second;
        // multiple children can be defined using the syntax: child1/child2/child3...
        // (add a slash so that each child is a string with a its name and a slash at the end)
        std::string childCombos = child->first.as<std::string>() + "/";
        while (childCombos.find("/") != std::string::npos) {
            std::string childName = childCombos.substr(0, childCombos.find("/"));
            addChild(structure, structurePath, childName, childAttributes["path"], spec);
            childCombos = childCombos.substr(childCombos.find("/") + 1);
        }
    }

    // apply rotation attribute to children
    for (YAML::const_iterator child = children.begin(); child != children.end(); ++child) {
        Yam childAttributes = child->second;
        // multiple children can be defined using the syntax: child1/child2/child3...
        // (add a slash so that each child is a string with a its name and a slash at the end)
        std::string childCombos = child->first.as<std::string>() + "/";
        while (childCombos.find("/") != std::string::npos) {
            std::string childName = childCombos.substr(0, childCombos.find("/"));
            tgStructure& childStructure = structure.findChild(childName);
            addChildRotation(childStructure, childAttributes["rotation"]);
            childCombos = childCombos.substr(childCombos.find("/") + 1);
        }
    }

    // apply scale, offset and translation attributes to children
    for (YAML::const_iterator child = children.begin(); child != children.end(); ++child) {
        Yam childAttributes = child->second;
        // multiple children can be defined using the syntax: child1/child2/child3...
        // (add a slash so that each child is a string with a its name and a slash at the end)
        std::string childCombos = child->first.as<std::string>() + "/";
        int childComboIndex = 0;
        while (childCombos.find("/") != std::string::npos) {
            std::string childName = childCombos.substr(0, childCombos.find("/"));
            tgStructure& childStructure = structure.findChild(childName);
            addChildScale(childStructure, childAttributes["scale"]);
            addChildOffset(childStructure, childComboIndex, childAttributes["offset"]);
            addChildTranslation(childStructure, childAttributes["translation"]);
            childCombos = childCombos.substr(childCombos.find("/") + 1);
            childComboIndex++;
        }
    }
}

void TensegrityModel::addChild(tgStructure& structure, const std::string &parentPath,
    const std::string& childName, const Yam& childStructurePath, tgBuildSpec& spec) {

    if (!childStructurePath) return;
    std::string childPath = childStructurePath.as<std::string>();
    // if path is relative, use path relative to parent structure
    if (childPath[0] != '/') {
        childPath = parentPath.substr(0, parentPath.rfind("/") + 1) + childPath;
    }
    tgStructure* childStructure = new tgStructure(childName);
    buildStructure(*childStructure, childPath, spec);
    structure.addChild(childStructure);
}

void TensegrityModel::addChildRotation(tgStructure& childStructure, const Yam& rotation) {
    if (!rotation) return;
    Yam reference = rotation["reference"];
    Yam axis = rotation["axis"];
    Yam angle = rotation["angle"];
    if (axis && angle) {
        double axisX = axis[0].as<double>();
        double axisY = axis[1].as<double>();
        double axisZ = axis[2].as<double>();
        btVector3 axisVector = btVector3(axisX, axisY, axisZ);
        double angleDegrees = angle.as<double>();
        double angleRadians = tgUtil::deg2rad(angleDegrees);
        btVector3 referenceVector;
        if (reference) {
            // rotate child around provided reference point
            double referenceX = reference[0].as<double>();
            double referenceY = reference[1].as<double>();
            double referenceZ = reference[2].as<double>();
            referenceVector = btVector3(referenceX, referenceY, referenceZ);
        }
        else {
            // rotate child around structure's centroid
            referenceVector = childStructure.getCentroid();
        }
        childStructure.addRotation(referenceVector, axisVector, angleRadians);
    }
}

void TensegrityModel::addChildScale(tgStructure& childStructure, const Yam& scale) {
    if (!scale) return;
    double scaleFactor = scale.as<double>();
    childStructure.scale(scaleFactor);
}

void TensegrityModel::addChildOffset(tgStructure& childStructure, int offsetIndex, const Yam& offset) {
    if (!offset) return;
    double offsetX = offset[0].as<double>();
    double offsetY = offset[1].as<double>();
    double offsetZ = offset[2].as<double>();
    btVector3 offsetVector = btVector3(offsetX, offsetY, offsetZ);
    childStructure.move(offsetVector * offsetIndex);
}

void TensegrityModel::addChildTranslation(tgStructure& childStructure, const Yam& translation) {
    if (!translation) return;
    double translationX = translation[0].as<double>();
    double translationY = translation[1].as<double>();
    double translationZ = translation[2].as<double>();
    btVector3 translationVector = btVector3(translationX, translationY, translationZ);
    childStructure.move(translationVector);
}

void TensegrityModel::buildStructure(tgStructure& structure, const std::string& structurePath, tgBuildSpec& spec) {
    Yam root = YAML::LoadFile(structurePath);
    addChildren(structure, structurePath, spec, root["substructures"]);
    addBuilders(spec, root["builders"]);
    addNodes(structure, root["nodes"]);
    addPairGroups(structure, root["pair_groups"]);
    addBondGroups(structure, root["bond_groups"], spec);
}

void TensegrityModel::addNodes(tgStructure& structure, const Yam& nodes) {
    if (!nodes) return;
    for (YAML::const_iterator node = nodes.begin(); node != nodes.end(); ++node) {
        std::string name = node->first.as<std::string>();
        Yam xyz = node->second;
        double x = xyz[0].as<double>();
        double y = xyz[1].as<double>();
        double z = xyz[2].as<double>();
        structure.addNode(x, y, z, name);
    }
}

void TensegrityModel::addPairGroups(tgStructure& structure, const Yam& pair_groups) {
    if (!pair_groups) return;
    for (YAML::const_iterator pair_group = pair_groups.begin(); pair_group != pair_groups.end(); ++pair_group) {
        std::string tags = pair_group->first.as<std::string>();
        Yam pairs = pair_group->second;
        addNodeNodePairs(structure, tags, pairs);
    }
}

void TensegrityModel::addBondGroups(tgStructure& structure, const Yam& bond_groups, tgBuildSpec& spec) {
    if (!bond_groups) return;

    // go through first level keys
    for (YAML::const_iterator firstLvl = bond_groups.begin(); firstLvl != bond_groups.end(); ++firstLvl) {
        std::string bonds;
        std::string tags;
        bool bondsSet = false;
        bool tagsSet = false;

        std::string firstLvlKey = firstLvl->first.as<std::string>();
        Yam firstLvlContent = firstLvl->second;
        // bonds must use the '/' notation
        if (firstLvlKey.find("/") == std::string::npos) {  // firstLvlKey is a tag
            tags = firstLvlKey;
            tagsSet = true;
        }
        else { // firstLvlKey is a bond
            bonds = firstLvlKey;
            bondsSet = true;
        }

        // go through second level keys
        for (YAML::const_iterator secondLvl = firstLvlContent.begin(); secondLvl != firstLvlContent.end(); ++secondLvl) {
            std::string secondLvlKey = secondLvl->first.as<std::string>();
            Yam secondLvlContent = secondLvl->second;
            if (secondLvlKey.find("/") == std::string::npos) { // secondLvlKey is a tag
                // a bond must be nested inside a tag or vice versa
                if (tagsSet) throw std::invalid_argument("Invalid nesting of tags: "+ secondLvlKey);
                tags = secondLvlKey;
            }
            else { // secondLvlKey is a bond
                if (bondsSet) throw std::invalid_argument("Invalid nesting of bonds: " + secondLvlKey);
                bonds = secondLvlKey;
            }
            addBonds(structure, bonds, tags, secondLvlContent, spec);
        }
    }
}

void TensegrityModel::addBonds(tgStructure& structure, const std::string& bonds, const std::string& tags, const Yam& pairs, tgBuildSpec& spec) {
    if (!pairs) return;
    // bonds looks like: childName1/childName2/.../bondType
    std::string bondType = bonds.substr(bonds.rfind("/") + 1);
    std::string structureCombos = bonds.substr(0, bonds.rfind("/"));
    if (structureCombos.find("/") == std::string::npos)
        throw std::invalid_argument("Error: all bond groups must specify a bond between 2 or more structures and a bond type");
    while (structureCombos.find("/") != std::string::npos) {
        std::string childStructure1Name = structureCombos.substr(0, structureCombos.find("/"));
        structureCombos = structureCombos.substr(structureCombos.find("/") + 1);
        std::string childStructure2Name = structureCombos.substr(0, structureCombos.find("/"));
        if (bondType == "node_node") {
            addNodeNodePairs(structure, tags, pairs, &childStructure1Name, &childStructure2Name);
        }
        else if (bondType == "node_edge") {
            addNodeEdgePairs(structure, tags, pairs, &childStructure1Name, &childStructure2Name, spec);
        }
        else {
            throw std::invalid_argument("Unsupported bond type: " + bondType);
        }
    }
}

void TensegrityModel::addNodeNodePairs(tgStructure& structure, const std::string& tags, const Yam& pairs,
    const std::string* childStructure1Name, const std::string* childStructure2Name) {

    for (YAML::const_iterator pairPtr = pairs.begin(); pairPtr != pairs.end(); ++pairPtr) {
        Yam pair = *pairPtr;
        std::string node1Path = pair[0].as<std::string>();
        std::string node2Path = pair[1].as<std::string>();
        tgNode* node1;
        tgNode* node2;
        if (childStructure1Name && childStructure2Name) {
            node1 = &getNode(structure.findChild(*childStructure1Name), node1Path);
            node2 = &getNode(structure.findChild(*childStructure2Name), node2Path);
        }
        else {
            node1 = &getNode(structure, node1Path);
            node2 = &getNode(structure, node2Path);
        }
        structure.addPair(*node1, *node2, tags);
    }
}

void TensegrityModel::addNodeEdgePairs(tgStructure& structure, const std::string& tags, const Yam& pairs,
    const std::string* childStructure1Name, const std::string* childStructure2Name, tgBuildSpec& spec) {

    if (pairs.size() < 3) {
        throw std::invalid_argument("Error: node_edge bonds must specify at least 3 node_edge pairs");
    }

    tgStructure& childStructure1 = structure.findChild(*childStructure1Name);
    tgStructure& childStructure2 = structure.findChild(*childStructure2Name);

    // these are used for transformations
    std::vector<btVector3> structure1RefNodes;
    std::vector<btVector3> structure2RefNodes;

    // these are nodes
    std::vector<tgNode*> ligands;
    // these are edges
    std::vector< std::pair<tgNode*, tgNode*> > receptors;

    // populate refNodes arrays, ligands and receptors
    parseNodeEdgePairs(childStructure1, childStructure2, structure1RefNodes, structure2RefNodes, ligands, receptors, pairs);
    rotateAndTranslate(childStructure2, structure1RefNodes, structure2RefNodes);

    std::vector<tgBuildSpec::RigidAgent*> rigidAgents = spec.getRigidAgents();
    for (unsigned int i = 0; i < ligands.size(); i++) {

        // remove old edge connections
        // try removing from both children since we are not sure which child the pair belongs to
        // (could add more information to receptors array so we don't have to do this)
        removePair(childStructure1, receptors[i].first, receptors[i].second, true, rigidAgents, spec);
        removePair(childStructure2, receptors[i].first, receptors[i].second, true, rigidAgents, spec);
        for (unsigned int j = 0; j < ligands.size(); j++) {
            // remove old string connections between nodes/ligands
            // try removing from both children since we are not sure which child the node belongs to
            removePair(childStructure1, ligands[i], ligands[j], false, rigidAgents, spec);
            removePair(childStructure2, ligands[i], ligands[j], false, rigidAgents, spec);
        }
        // make new connection from edge -> node -> edge
        structure.addPair(*(receptors[i].first), *ligands[i], tags);
        structure.addPair(*ligands[i], *(receptors[i].second), tags);

    }
}

void TensegrityModel::parseNodeEdgePairs(tgStructure& childStructure1, tgStructure& childStructure2,
    std::vector<btVector3>& structure1RefNodes, std::vector<btVector3>& structure2RefNodes,
    std::vector<tgNode*>& ligands, std::vector< std::pair<tgNode*, tgNode*> >& receptors, const Yam& pairs) {

    for (YAML::const_iterator pairPtr = pairs.begin(); pairPtr != pairs.end(); ++pairPtr) {
        Yam pair = *pairPtr;

        std::string structure1Attachment = pair[0].as<std::string>();
        std::string structure2Attachment = pair[1].as<std::string>();

        // node_edge bond must be between a node and an edge (an edge will be defined as 'node1/node2')
        if (structure1Attachment.find("/") == std::string::npos && structure2Attachment.find("/") == std::string::npos) {
            throw std::invalid_argument("Invalid node_edge bond: " + structure1Attachment + ", " + structure2Attachment);
        }
        if (structure1Attachment.find("/") != std::string::npos && structure2Attachment.find("/") != std::string::npos) {
            throw std::invalid_argument("Invalid node_edge bond: " + structure1Attachment + ", " + structure2Attachment);
        }
        parseAttachmentPoint(childStructure1, structure1Attachment, structure1RefNodes, ligands, receptors);
        parseAttachmentPoint(childStructure2, structure2Attachment, structure2RefNodes, ligands, receptors);
    }
}

void TensegrityModel::parseAttachmentPoint(tgStructure& structure, const std::string& attachment, std::vector<btVector3>& refNodes,
    std::vector<tgNode*>& ligands, std::vector< std::pair<tgNode*, tgNode*> >& receptors) {
    if (attachment.find("/") == std::string::npos) { // node
        tgNode* attachmentNode = &(getNode(structure, attachment));
        ligands.push_back(attachmentNode);
        refNodes.push_back(*attachmentNode);
    }
    else {  // edge
        std::string attachmentNode1Path = attachment.substr(0, attachment.find("/"));
        std::string attachmentNode2Path = attachment.substr(attachment.find("/") + 1);

        tgNode* attachmentNode1 = &(getNode(structure, attachmentNode1Path));
        tgNode* attachmentNode2 = &(getNode(structure, attachmentNode2Path));

        receptors.push_back(std::make_pair(attachmentNode1, attachmentNode2));
        refNodes.push_back((*attachmentNode1 + *attachmentNode2) / 2);
    }
}

// todo: should use a best fit transformation from one set of points to another
void TensegrityModel::rotateAndTranslate(tgStructure& childStructure2,
    std::vector<btVector3>& structure1RefNodes, std::vector<btVector3>& structure2RefNodes) {

    btVector3 structure1RefNodesCentroid = tgUtil::getCentroid(structure1RefNodes);
    btVector3 structure2RefNodesCentroid = tgUtil::getCentroid(structure2RefNodes);

    btVector3 structure1PlaneNormal = ((structure1RefNodes[1] - structure1RefNodes[0]).
        cross(structure1RefNodes[2] - structure1RefNodes[0])).normalize();
    btVector3 structure2PlaneNormal = ((structure2RefNodes[1] - structure2RefNodes[0]).
        cross(structure2RefNodes[2] - structure2RefNodes[0])).normalize();

    // rotate structure 2 to align normals
    btVector3 fallBackAxis = (structure2RefNodes[1] - structure2RefNodes[0]).normalize();
    childStructure2.addRotation(structure2RefNodesCentroid,
        tgUtil::getQuaternionBetween(structure2PlaneNormal, structure1PlaneNormal, fallBackAxis));

    // rotate structure 2 ref nodes
    tgUtil::addRotation(structure2RefNodes[0], structure2RefNodesCentroid,
        tgUtil::getQuaternionBetween(structure2PlaneNormal, structure1PlaneNormal, fallBackAxis));
    tgUtil::addRotation(structure2RefNodesCentroid, structure2RefNodesCentroid,
        tgUtil::getQuaternionBetween(structure2PlaneNormal, structure1PlaneNormal, fallBackAxis));

    // translate structure 2 to match up centroid points
    childStructure2.move(structure1RefNodesCentroid - structure2RefNodesCentroid);

    // translate structure 2 ref nodes
    structure2RefNodes[0] += structure1RefNodesCentroid - structure2RefNodesCentroid;
    structure2RefNodesCentroid += structure1RefNodesCentroid - structure2RefNodesCentroid;

    // rotate structure 2 around structure1PlaneNormal axis to match up node with edge midpoints
    childStructure2.addRotation(structure1RefNodesCentroid,
        tgUtil::getQuaternionBetween(structure2RefNodes[0] - structure1RefNodesCentroid,
        structure1RefNodes[0] - structure1RefNodesCentroid, structure1PlaneNormal));
}


void TensegrityModel::removePair(tgStructure& structure, const tgNode* from, const tgNode* to, bool isEdgePair,
    const std::vector<tgBuildSpec::RigidAgent*>& rigidAgents, tgBuildSpec& spec) {

    tgPair *pair;
    try {
        pair = &structure.findPair(*from, *to);
    } catch (std::invalid_argument e) {
        return;
    }
    for (int i = 0; i < rigidAgents.size(); i++) {
        tgTagSearch tagSearch = rigidAgents[i]->tagSearch;
        if (tagSearch.matches(*pair)) {
            // don't want to remove pairs that are rods
            if (isEdgePair) {
                throw std::invalid_argument("Edges in node_edge bonds cannot be rods");
            }
            return;
        }
    }
    structure.removePair(*pair);
}

tgNode& TensegrityModel::getNode(tgStructure& structure, const std::string& nodePath) {
    // nodePath looks like: 'parentStructure.childStructure.nodeName'
    if (nodePath.find(".") == std::string::npos) {
        return structure.findNode(nodePath);
    }
    else {
        std::string structurePath = nodePath.substr(0, nodePath.rfind("."));
        std::string nodeName = nodePath.substr(nodePath.rfind(".") + 1);
        tgStructure& targetStructure = getStructure(structure, structurePath);
        return targetStructure.findNode(nodeName);
    }
}

tgStructure& TensegrityModel::getStructure(tgStructure& parentStructure, const std::string& childStructurePath) {
    // childStructurePath looks like: 'parentStructure.childStructure'
    if (childStructurePath.find(".") == std::string::npos) {
        return parentStructure.findChild(childStructurePath);
    }
    else  {
        std::string childStructureName = childStructurePath.substr(0, childStructurePath.rfind("."));
        std::string remainingChildStructurePath = childStructurePath.substr(childStructurePath.rfind(".") + 1);
        tgStructure& childStructure = parentStructure.findChild(childStructureName);
        return getStructure(childStructure, remainingChildStructurePath);
    }
}

void TensegrityModel::addBuilders(tgBuildSpec& spec, const Yam& builders) {
    for (YAML::const_iterator builder = builders.begin(); builder != builders.end(); ++builder) {

        std::string tagMatch = builder->first.as<std::string>();
        std::string builderClass = builder->second["class"].as<std::string>();
        Yam parameters = builder->second["parameters"];

        if (builderClass == "tgRodInfo") {
            addRodBuilder(builderClass, tagMatch, parameters, spec);
        }
        else if (builderClass == "tgBasicActuatorInfo" || builderClass == "tgBasicContactCableInfo") {
            addBasicActuatorBuilder(builderClass, tagMatch, parameters, spec);
        }
        else if (builderClass == "tgKinematicContactCableInfo" || builderClass == "tgKinematicActuatorInfo") {
            addKinematicActuatorBuilder(builderClass, tagMatch, parameters, spec);
        }
        // add more builders here if they use a different Config
        else {
            throw std::invalid_argument("Unsupported builder class: " + builderClass);
        }
    }
}

void TensegrityModel::addRodBuilder(const std::string& builderClass, const std::string& tagMatch, const Yam& parameters, tgBuildSpec& spec) {
    // rodParameters
    std::map<std::string, double> rp;
    rp["radius"] = rodRadius;
    rp["density"] = rodDensity;
    rp["friction"] = rodFriction;
    rp["roll_friction"] = rodRollFriction;
    rp["restitution"] = rodRestitution;

    if (parameters) {
        for (YAML::const_iterator parameter = parameters.begin(); parameter != parameters.end(); ++parameter) {
            std::string parameterName = parameter->first.as<std::string>();
            if (rp.find(parameterName) == rp.end()) {
                throw std::invalid_argument("Unsupported " + builderClass + " parameter: " + parameterName);
            }
            // if defined overwrite default parameter value
            rp[parameterName] = parameter->second.as<double>();
        }
    }

    const tgRod::Config* rodConfig = new tgRod::Config(rp["radius"], rp["density"], rp["friction"],
        rp["roll_friction"], rp["restitution"]);
    if (builderClass == "tgRodInfo") {
        spec.addBuilder(tagMatch, new tgRodInfo(*rodConfig));
    }
    // add more builders that use tgRod::Config here
}

void TensegrityModel::addBasicActuatorBuilder(const std::string& builderClass, const std::string& tagMatch, const Yam& parameters, tgBuildSpec& spec) {
    // basicActuatorParameters
    std::map<std::string, double> bap;
    bap["stiffness"] = stringStiffness;
    bap["damping"] = stringDamping;
    bap["pretension"] = stringPretension;
    bap["history"] = stringHistory;
    bap["max_tension"] = stringMaxTension;
    bap["target_velocity"] = stringTargetVelocity;
    bap["min_actual_length"] = stringMinActualLength;
    bap["min_rest_length"] = stringMinRestLength;
    bap["rotation"] = stringRotation;

    if (parameters) {
        for (YAML::const_iterator parameter = parameters.begin(); parameter != parameters.end(); ++parameter) {
            std::string parameterName = parameter->first.as<std::string>();
            if (bap.find(parameterName) == bap.end()) {
                throw std::invalid_argument("Unsupported " + builderClass + " parameter: " + parameterName);
            }
            // if defined overwrite default parameter value
            bap[parameterName] = parameter->second.as<double>();
        }
    }

    const tgBasicActuator::Config* basicActuatorConfig = new tgBasicActuator::Config(bap["stiffness"],
        bap["damping"], bap["pretension"], bap["history"], bap["max_tension"], bap["target_velocity"],
        bap["min_actual_length"], bap["min_rest_length"], bap["rotation"]);
    if (builderClass == "tgBasicActuatorInfo") {
        spec.addBuilder(tagMatch, new tgBasicActuatorInfo(*basicActuatorConfig));
    }
    else if (builderClass == "tgBasicContactCableInfo") {
        spec.addBuilder(tagMatch, new tgBasicContactCableInfo(*basicActuatorConfig));
    }
    // add more builders that use tgBasicActuator::Config here
}

void TensegrityModel::addKinematicActuatorBuilder(const std::string& builderClass, const std::string& tagMatch, const Yam& parameters, tgBuildSpec& spec) {
    // kinematicActuatorParameters
    std::map<std::string, double> kap;
    kap["stiffness"] = stringStiffness;
    kap["damping"] = stringDamping;
    kap["pretension"] = stringPretension;
    kap["radius"] = stringRadius;
    kap["motor_friction"] = stringMotorFriction;
    kap["motor_inertia"] = stringMotorInertia;
    kap["back_drivable"] = stringBackDrivable;
    kap["history"] = stringHistory;
    kap["max_tension"] = stringMaxTension;
    kap["target_velocity"] = stringTargetVelocity;
    kap["min_actual_length"] = stringMinActualLength;
    kap["min_rest_length"] = stringMinRestLength;
    kap["rotation"] = stringRotation;

    if (parameters) {
        for (YAML::const_iterator parameter = parameters.begin(); parameter != parameters.end(); ++parameter) {
            std::string parameterName = parameter->first.as<std::string>();
            if (kap.find(parameterName) == kap.end()) {
                throw std::invalid_argument("Unsupported " + builderClass + " parameter: " + parameterName);
            }
            // if defined overwrite default parameter value
            kap[parameterName] = parameter->second.as<double>();
        }
    }

    const tgKinematicActuator::Config* kinematicActuatorConfig =
        new tgKinematicActuator::Config(kap["stiffness"], kap["damping"], kap["pretension"], kap["radius"],
        kap["motor_friction"], kap["motor_inertia"],  kap["back_drivable"], kap["history"], kap["max_tension"],
        kap["target_velocity"],kap["min_actual_length"], kap["min_rest_length"], kap["rotation"]);
    if (builderClass == "tgKinematicContactCableInfo") {
        spec.addBuilder(tagMatch, new tgKinematicContactCableInfo(*kinematicActuatorConfig));
    }
    else if (builderClass == "tgKinematicActuatorInfo") {
        spec.addBuilder(tagMatch, new tgKinematicActuatorInfo(*kinematicActuatorConfig));
    }
    // add more builders that use tgKinematicActuator::Config here
}

void TensegrityModel::step(double timeStep) {
    if (timeStep <= 0.0) {
        throw std::invalid_argument("time step is not positive");
    }
    else {
        notifyStep(timeStep);
        tgModel::step(timeStep);
    }
}

void TensegrityModel::onVisit(tgModelVisitor& visitor) {
    tgModel::onVisit(visitor);
}

const std::vector<tgSpringCableActuator*>& TensegrityModel::getAllActuators() const {
    return allActuators;
}

void TensegrityModel::teardown() {
    notifyTeardown();
    tgModel::teardown();
}
