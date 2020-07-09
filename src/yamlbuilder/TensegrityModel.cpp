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
 * @author Simon Kotwicz, Jonah Eisen, Drew Sabelhaus
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
#include "core/tgBox.h"
#include "core/tgSphere.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgBasicContactCableInfo.h"
#include "tgcreator/tgKinematicActuatorInfo.h"
#include "tgcreator/tgKinematicContactCableInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgBoxInfo.h"
#include "tgcreator/tgSphereInfo.h"
#include "tgcreator/tgStructureInfo.h"

/**
 * Constructor that only takes the path to the YAML file.
 */
TensegrityModel::TensegrityModel(const std::string& structurePath) : tgModel() {
    topLvlStructurePath = structurePath;
}

/**
 * Constructor that includes the debugging flag.
 */
TensegrityModel::TensegrityModel(const std::string& structurePath,
				 bool debugging) : tgModel() {
    topLvlStructurePath = structurePath;
    // All places in this file controlled by 'debugging_on' are labelled
    // with comments with the string DEBUGGING.
    debugging_on = debugging;
}

TensegrityModel::~TensegrityModel() {}

/**
 * Debugging function. Outputs the tgStructure, tgStructureInfo, and tgModel,
 * as created by this class.
 */
void TensegrityModel::trace(const tgStructure& structure,
			    const tgStructureInfo& structureInfo, tgModel& model)
{
    std::cout << std::endl << "Structure Trace inside TensegrityModel:" << std::endl
    << structure        << std::endl 
    << std::endl << "StructureInfo Trace inside TensegrityModel:" << std::endl
    << structureInfo    << std::endl
    << std::endl << "tgModel Trace inside Tensegrity Model: " << std::endl
    << model            << std::endl;
}

/**
 * The setup function is what's called from outside this class.
 * It is responsible for creating all the parts of this tgModel and
 * calling the tgStructureInfo to build the structure into the world.
 */
void TensegrityModel::setup(tgWorld& world) {
    // create the build spec that uses tags to turn the structure into a model
    tgBuildSpec spec;

    // add default builders (rods, strings, boxes) that match the tags (rods, strings, boxes, spheres)
    // (these will be overwritten if a different builder is specified for those tags)
    Yam emptyYam = Yam();
    addRodBuilder("tgRodInfo", "rod", emptyYam, spec);
    addBasicActuatorBuilder("tgBasicActuatorInfo", "string", emptyYam, spec);
    addBoxBuilder("tgBoxInfo", "box", emptyYam, spec);
    addSphereBuilder("tgSphereInfo", "sphere", emptyYam, spec);

    tgStructure structure;
    buildStructure(structure, topLvlStructurePath, spec);

    tgStructureInfo structureInfo(structure, spec);
    structureInfo.buildInto(*this, world);

    // use tgCast::filterto pull out the muscles that we want to control
    allActuators = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());

    // DEBUGGING: print out the tgStructure, tgStructureInfo, and tgModel.
    if(debugging_on) {
        trace(structure, structureInfo, *this);
    }

    // notify controllers that setup has finished
    notifySetup();

    // actually setup the children
    tgModel::setup(world);
}

void TensegrityModel::addChildren(tgStructure& structure, const std::string& structurePath, tgBuildSpec& spec, const Yam& children) {
    if (!children) return;
    std::string structureAttributeKeys[] = {"path", "rotation", "translation", "scale", "offset"};
    std::vector<std::string> structureAttributeKeysVector(structureAttributeKeys, structureAttributeKeys + sizeof(structureAttributeKeys) / sizeof(std::string));

    // add all the children first
    for (YAML::const_iterator child = children.begin(); child != children.end(); ++child) {
        Yam childAttributes = child->second;
        yamlContainsOnly(childAttributes, structurePath, structureAttributeKeysVector);
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
    tgStructure childStructure = tgStructure(childName);
    buildStructure(childStructure, childPath, spec);
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
    /** 
     * This call to YAML::LoadFile can return the exception YAML::BadFile 
     * if any of the yaml files or substructure files cannot be found. 
     * Make this error more explicit through a try and catch.
     */
    Yam root;
    try
    {
      root = YAML::LoadFile(structurePath);
    }
    catch( YAML::BadFile badfileexception )
    {
      // If a BadFile exception is thrown, output a detailed message first:
      std::cout << std::endl << "The YAML parser threw a BadFile exception when" <<
	" trying to load one of your YAML files. " << std::endl <<
	"The path of the structure that the parser attempted to load is: '" <<
	structurePath << "'. " << std::endl <<
	"Check to be sure that the file exists, and " <<
	"that you didn't spell the path name incorrectly." <<
	std::endl << std::endl;
      // Then, throw the exception again, so that the program stops.
      throw badfileexception;
    }
    // Validate YAML
    std::string rootKeys[] = {"nodes", "pair_groups", "builders", "substructures", "bond_groups"};
    std::vector<std::string> rootKeysVector(rootKeys, rootKeys + sizeof(rootKeys) / sizeof(std::string));
    yamlContainsOnly(root, structurePath, rootKeysVector);
    yamlNoDuplicates(root, structurePath);

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
	  // DEBUGGING: output the bonds that are about to be created.
	  if(debugging_on) {
	      std::cout << "Adding node_node bonds " << tags <<
		" between structures " << childStructure1Name <<
		" and " << childStructure2Name << std::endl;
	  }
	  // Add the node_node pairs, regardless of debugging
	  addNodeNodePairs(structure, tags, pairs, &childStructure1Name,
	    &childStructure2Name);
        }
        else if (bondType == "node_edge") {
            addNodeEdgePairs(structure, tags, pairs, &childStructure1Name, &childStructure2Name, spec);
        }
        else {
            throw std::invalid_argument("Unsupported bond type: " + bondType);
        }
    }
}

void TensegrityModel::addNodeNodePairs(tgStructure& structure,
	    const std::string& tags, const Yam& pairs,
	    const std::string* childStructure1Name,
	    const std::string* childStructure2Name) {
	  
  // NOTE that this method can be called with the childStructureName pointers
  // equal 0. That's the case if there is no child structure (e.g., this method
  // is adding a pair to a base structure.)
  
  //DEBUGGING: output the node_node pairs about to be created.
  if(debugging_on) {
    // This statement is true if the pointer is nonzero.
    if (childStructure1Name && childStructure2Name) {
      std::cout << "Adding all the node_node pairs between structures "
		<< *childStructure1Name << " and " << *childStructure2Name
		<< std::endl;
    }
    else {
      std::cout << "Adding all the node_node pairs within a base structure." <<
	std::endl;
    }
  }
  // Iterate over the pairs and add them
  for (YAML::const_iterator pairPtr = pairs.begin(); pairPtr != pairs.end(); ++pairPtr) {
    Yam pair = *pairPtr;
    std::string node1Path = pair[0].as<std::string>();
    std::string node2Path = pair[1].as<std::string>();
    tgNode* node1;
    tgNode* node2;
    // This method may add additional tags. So, make a new variable here
    // and then change it later if needed.
    std::string pairNewTags = tags;
    // This statement is true if the pointer is nonzero.
    if (childStructure1Name && childStructure2Name) {
      node1 = &getNode(structure.findChild(*childStructure1Name), node1Path);
      node2 = &getNode(structure.findChild(*childStructure2Name), node2Path);
      // DEBUGGING: List the specific pairs about to be added
      if(debugging_on) {
	std::cout << "Adding node_node pair " << tags << " between structures "
		  << *childStructure1Name << " and " << *childStructure2Name
		  << " for nodes " << *node1 << " and " << *node2
		  << " with original tag " << tags << std::endl;
      }
      // Add three additional tags: the names of the two structures that
      // a pair connects. This is useful for controllers, where tags are used
      // to designate one actuator from another.
      // As per tgTaggable, tags are separated by spaces.
      // Three tags are added: the two connecting structure names, and the
      // two names connected by a slash, like in the YAML file.
      pairNewTags = tags + " " + *childStructure1Name + " " + *childStructure2Name
	+ " " + *childStructure1Name + "/" + *childStructure2Name;
    }
    else {
      // Pointers are zero, add directly to this structure and not any
      // of its children (since there ARE no children!)
      node1 = &getNode(structure, node1Path);
      node2 = &getNode(structure, node2Path);
    }
    // finally, add the actual pair.
    //structure.addPair(*node1, *node2, tags);
    structure.addPair(*node1, *node2, pairNewTags);
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

/// @todo should use a best fit transformation from one set of points to another
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
        if (!builder->second["class"]) throw std::invalid_argument("Builder class not supplied for tag: " + tagMatch);
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
	else if (builderClass == "tgBoxInfo") {
            addBoxBuilder(builderClass, tagMatch, parameters, spec);
        }
	else if (builderClass == "tgSphereInfo") {
	    addSphereBuilder(builderClass, tagMatch, parameters, spec);
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

    const tgRod::Config rodConfig = tgRod::Config(rp["radius"], rp["density"], rp["friction"],
        rp["roll_friction"], rp["restitution"]);
    if (builderClass == "tgRodInfo") {
        // tgBuildSpec takes ownership of the tgRodInfo object
        spec.addBuilder(tagMatch, new tgRodInfo(rodConfig));
    }
    // add more builders that use tgRod::Config here
}

void TensegrityModel::addBasicActuatorBuilder(const std::string& builderClass, const std::string& tagMatch, const Yam& parameters, tgBuildSpec& spec) {
    // tgbBasicActuator parameters.
    // This method assigns default values based on TensegrityModel.h,
    // then overwrites them if a parameter is specified in the YAML file.
    // See tgBasicActuator.h for a description of each parameter.
    // Since tgBasicActuator has both double and boolean config parameters,
    // store two lists of parameters based on type.
    // We use the abbreviation "bap" as basic actuator parameters.
    std::map<std::string, double> bap_doubles;
    std::map<std::string, bool> bap_booleans;
    bap_doubles["stiffness"] = stringStiffness;
    bap_doubles["damping"] = stringDamping;
    bap_doubles["pretension"] = stringPretension;
    bap_doubles["history"] = stringHistory;
    bap_doubles["max_tension"] = stringMaxTension;
    bap_doubles["target_velocity"] = stringTargetVelocity;
    bap_doubles["min_actual_length"] = stringMinActualLength;
    bap_doubles["min_rest_length"] = stringMinRestLength;
    bap_doubles["rotation"] = stringRotation;
    bap_booleans["moveCablePointAToEdge"] = stringMoveCablePointAToEdge;
    bap_booleans["moveCablePointBToEdge"] = stringMoveCablePointBToEdge;

    // If no parameters are passed in, do not change anything.
    if (parameters) {
        // Iterate through all the parameters passed in for this builder.
        for (YAML::const_iterator parameter = parameters.begin(); parameter != parameters.end(); ++parameter) {
	    // The key for both maps is a string.
	    std::string parameterName = parameter->first.as<std::string>();
	    // However, the value may be either a double or a boolean. Check both
	    // lists, and mark a flag depending on the output.
	    bool paramIsDouble = true;
	    // Check the doubles first:
            if( bap_doubles.find(parameterName) == bap_doubles.end()) {
	        // The parameter is not a double, since it's not in the doubles map.
  	        paramIsDouble = false;
	        // Then, check to see if the parameter is in the booleans map.
		if( bap_booleans.find(parameterName) == bap_booleans.end()) {
		  // The parameter is in neither list, throw an exception.
		  throw std::invalid_argument("Unsupported " + builderClass + " parameter: " + parameterName);
		}
            }
            // if defined, overwrite default parameter value to the appropriate map.
	    if( paramIsDouble ) {
	      // change the value in the doubles list
	      bap_doubles[parameterName] = parameter->second.as<double>();
	    }
	    else {
	      // the value is in the booleans list.
	      bap_booleans[parameterName] = parameter->second.as<bool>();
	    }
        }
    }

    // Create the config struct.
    // Note that this calls the constructor for Config, so the parameters
    // are passed in according to order not name.
    // @TO-DO: instead, create a Config with nothing passed in, and then change
    // parameters if defined. That way, no defaults would need to be defined
    // in TensegrityModel.h. Currently, defaults are defined in BOTH the
    // actual config struct definition as well as in this .h file.
    const tgBasicActuator::Config basicActuatorConfig =
      tgBasicActuator::Config(bap_doubles["stiffness"], bap_doubles["damping"],
			      bap_doubles["pretension"], bap_doubles["history"],
			      bap_doubles["max_tension"],
			      bap_doubles["target_velocity"],
			      bap_doubles["min_actual_length"],
			      bap_doubles["min_rest_length"],
			      bap_doubles["rotation"],
			      bap_booleans["moveCablePointAToEdge"],
			      bap_booleans["moveCablePointBToEdge"]);
    if (builderClass == "tgBasicActuatorInfo") {
        // tgBuildSpec takes ownership of the tgBasicActuatorInfo object
        spec.addBuilder(tagMatch, new tgBasicActuatorInfo(basicActuatorConfig));
    }
    else if (builderClass == "tgBasicContactCableInfo") {
        // tgBuildSpec takes ownership of the tgBasicContactCableInfo object
        spec.addBuilder(tagMatch, new tgBasicContactCableInfo(basicActuatorConfig));
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

    const tgKinematicActuator::Config kinematicActuatorConfig =
        tgKinematicActuator::Config(kap["stiffness"], kap["damping"], kap["pretension"], kap["radius"],
        kap["motor_friction"], kap["motor_inertia"],  kap["back_drivable"], kap["history"], kap["max_tension"],
        kap["target_velocity"],kap["min_actual_length"], kap["min_rest_length"], kap["rotation"]);
    if (builderClass == "tgKinematicContactCableInfo") {
        // tgBuildSpec takes ownership of the tgKinematicContactCableInfo object
        spec.addBuilder(tagMatch, new tgKinematicContactCableInfo(kinematicActuatorConfig));
    }
    else if (builderClass == "tgKinematicActuatorInfo") {
        // tgBuildSpec takes ownership of the tgKinematicActuatorInfo object
        spec.addBuilder(tagMatch, new tgKinematicActuatorInfo(kinematicActuatorConfig));
    }
    // add more builders that use tgKinematicActuator::Config here
}

void TensegrityModel::addBoxBuilder(const std::string& builderClass, const std::string& tagMatch, const Yam& parameters, tgBuildSpec& spec) {
    /**
     * Builder procedure: 
     * (1) create a list (a map, really) of all the possible parameters, and initialize
     * them to the defaults in TensegrityModel.h
     * (2) substitute these defaults with any parameters from the YAMl file
     * (3) create the tgBoxInfo object and add it to the list of builders
     * @TO-DO: this seems to be called multiple times per "builder" block in a YAML file.
     * That's probably not correct. Why?
     */

    // (1)
    // Parameters to be used in tgBox::Config. See core/tgBox.h
    // Boxes are treated like rods with a rectangular cross-section: the box length
    // is specified by the distance between nodes, and the width and height come from
    // the Config struct.
    // Create a map of strings to doubles that will hold the parameters
    std::map<std::string, double> bp;
    bp["width"] = boxWidth;
    bp["height"] = boxHeight;
    bp["density"] = boxDensity;
    bp["friction"] = boxFriction;
    bp["roll_friction"] = boxRollFriction;
    bp["restitution"] = boxRestitution;
    
    if (parameters) {
        for (YAML::const_iterator parameter = parameters.begin(); parameter != parameters.end(); ++parameter) {
            std::string parameterName = parameter->first.as<std::string>();
            if (bp.find(parameterName) == bp.end()) {
                throw std::invalid_argument("Unsupported " + builderClass + " parameter: " + parameterName);
            }
            // if defined overwrite default parameter value
            bp[parameterName] = parameter->second.as<double>();
        }
    }

    // (3)
    // this usage is the same as in NTRT v1.0 models.
    const tgBox::Config boxConfig = tgBox::Config(bp["width"], bp["height"],
			 bp["density"], bp["friction"], bp["roll_friction"], bp["restitution"]);
    if (builderClass == "tgBoxInfo") {
        // tgBuildSpec takes ownership of the tgBoxInfo object
        spec.addBuilder(tagMatch, new tgBoxInfo(boxConfig));
    }
}

void TensegrityModel::addSphereBuilder(const std::string& builderClass, const std::string& tagMatch, const Yam& parameters, tgBuildSpec& spec){
  /**
   * Builder prcedure:
   * (1) create list of all possible parameters, as with box (for example)
   * (2) substitute in any passed-in paremeters in the YAML file
   * (3) VALIDATION: for the sphere, we must demand that BOTH NODES be the same.
   *     This is a pretty terrible hack at the moment, but the YAML parser only
   *     supports two-node objects, whereas the sphere is a single-node object.
   * (4) create the tgSphereInfo object and add it to the list of builders.
   */

  // (1)
  // Parameters to be used in tgSphere::Config. See core/tgSphere.h
  // Spheres are simple: one point, one radius, and then same
  // rigid body parameters as the rest of the gang.
  // Create a map of strings to doubles that will hold the sphere parameters
  std::map<std::string, double> sp;
  sp["radius"] = sphereRadius;
  sp["density"] = sphereDensity;
  sp["friction"] = sphereFriction;
  sp["roll_friction"] = sphereRollFriction;
  sp["restitution"] = sphereRestitution;

  // (2) sub in the new stuff if any exists
  if (parameters) {
    for (YAML::const_iterator parameter = parameters.begin(); parameter != parameters.end(); ++parameter) {
      std::string parameterName = parameter->first.as<std::string>();
      if (sp.find(parameterName) == sp.end()) {
	throw std::invalid_argument("Unsupported " + builderClass + " parameter: " + parameterName);
      }
      // if defined overwrite default parameter value
      sp[parameterName] = parameter->second.as<double>();
    }
  }

  // (3)
  // VALIDATION. We need the node locations to be the same.
  // Can't create a sphere in two different places.
  // This is a hack. To-do: refactor the YAML parser so that builders can be
  // created for a single node, or more than two nodes.
  // ...actually, we can't do this check here. Makes things more difficult...

  // (4) add the builder. HOPEFULLY, this will create a builder that only
  // looks for nodes, not pairs. To do, check tgSphereInfo to see what happens if
  // it encounters a pair of spheres.
  const tgSphere::Config sphereConfig = tgSphere::Config(sp["radius"],
		 sp["density"], sp["friction"], sp["roll_friction"], sp["restitution"]);
  if (builderClass == "tgSphereInfo") {
    // tgBuildSpec takes ownership of the tgSphereInfo object
    spec.addBuilder(tagMatch, new tgSphereInfo(sphereConfig));
  }
  
}

void TensegrityModel::yamlNoDuplicates(const Yam& yam, const std::string structurePath) {
    std::set<std::string> keys;
    for (YAML::const_iterator iter = yam.begin(); iter != yam.end(); ++iter) {
        Yam child = iter->second;
        if (child.Type() == YAML::NodeType::Map) {
            yamlNoDuplicates(child, structurePath);
        }
        std::string keyName = iter->first.as<std::string>();
        if (keys.find(keyName) == keys.end()) {
            keys.insert(keyName);
        }
        else {
            throw std::invalid_argument(structurePath.substr(structurePath.rfind("/") + 1) + " contains duplicate key: " + keyName);
        }
    }
}

void TensegrityModel::yamlContainsOnly(const Yam& yam, const std::string structurePath, const std::vector<std::string> keys) {
    for (YAML::const_iterator key = yam.begin(); key != yam.end(); ++key) {
        std::string keyName = key->first.as<std::string>();
        if (std::find(keys.begin(), keys.end(), keyName) == keys.end()) {
            throw std::invalid_argument(structurePath.substr(structurePath.rfind("/") + 1) + " contains invalid key: " + keyName);
        }
    }
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
