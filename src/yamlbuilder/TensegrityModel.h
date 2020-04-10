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

#ifndef TENSEGRITY_MODEL_H
#define TENSEGRITY_MODEL_H

/**
 * @file TensegrityModel.cpp
 * @brief Contains the definition of the members of the class TensegrityModel.
 * @author Simon Kotwicz, Jonah Eisen, Drew Sabelhaus
 * @copyright Copyright (C) 2016 NASA Ames Research Center
 * $Id$
 */

// C++ Standard Library
#include <map>
#include <string>
#include <vector>
// NTRT Core and tgCreator Libraries
#include "core/tgModel.h"
#include "core/tgSubject.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgStructure.h"
// Bullet Physics library
#include "LinearMath/btVector3.h"
// Helper libraries
#include <yaml-cpp/yaml.h>

// Forward declarations
class tgSpringCableActuator;
class tgModelVisitor;
class tgWorld;
class tgStructureInfo;

typedef YAML::Node Yam; // to avoid confusion with structure nodes

/**
 * A class that constructs a model based on a tensegrity structure encoded in YAML.
 */
class TensegrityModel : public tgSubject<TensegrityModel>, public tgModel
{
public:

    /**
     * List of all the default config parameters for all the
     * types of objects that wil be supported.
     */

    // Rod parameters:
  
    /*
     * Default rod radius.
     */
    const static double rodRadius = 0.5;
    /*
     * Default rod density.
     */
    const static double rodDensity = 1.0;
    /*
     * Default rod friction.
     */
    const static double rodFriction = 1.0;
    /*
     * Default rod roll friction.
     */
    const static double rodRollFriction = 0.0;
    /*
     * Default rod restitution.
     */
    const static double rodRestitution = 0.2;

    // Box parameters:
  
    /*
     * Default box width. (see tgBox.h)
     */
    const static double boxWidth = 1.0;
    /*
     * Default box height. (see tgBox.h)
     */
    const static double boxHeight = 1.0;
    /*
     * Default box density. (see tgBox.h)
     */
    const static double boxDensity = 1.0;
    /*
     * Default box friction. (see tgBox.h)
     */
    const static double boxFriction = 1.0;
    /*
     * Default box rolling friction. (see tgBox.h)
     */
    const static double boxRollFriction = 0.0;
    /*
     * Default box restitution. (see tgBox.h)
     */
    const static double boxRestitution = 0.2;

    // Sphere parameters:
    
    /*
     * Default sphere radius, density, friction, rolling friction, density, etc.
     * See tgSphere.h.
     */
    const static double sphereRadius = 0.5;
    const static double sphereDensity = 1.0;
    const static double sphereFriction = 1.0;
    const static double sphereRollFriction = 0.0;
    const static double sphereRestitution = 0.2;

    // String parameters:
    
    /*
     * Default string stiffness.
     */
    const static double stringStiffness = 1000.0;
    /*
     * Default string damping.
     */
    const static double stringDamping = 10.0;
    /*
     * Default string pretension.
     */
    const static double stringPretension = 0.0;
    /*
     * Default string radius.
     */
    const static double stringRadius = 1.0;
    /*
     * Default string motor friction.
     */
    const static double stringMotorFriction = 0.0;
    /*
     * Default string motor intertia.
     */
    const static double stringMotorInertia = 1.0;
    /*
     * Default string back drivable (boolean).
     */
    const static double stringBackDrivable = 0;
    /*
     * Default string history (boolean).
     */
    const static double stringHistory = 0;
    /*
     * Default string max tension.
     */
    const static double stringMaxTension = 1000.0;
    /*
     * Default string target velocity.
     */
    const static double stringTargetVelocity = 100.0;
    /*
     * Default string min actual length.
     */
    const static double stringMinActualLength = 0.1;
    /*
     * Default string min rest length.
     */
    const static double stringMinRestLength = 0.1;
    /*
     * Default string rotation.
     */
    const static double stringRotation = 0;
    /*
     * Default flags for automatic movement of cable anchor points.
     * See the config struct in tgBasicActuator.h for more information.
     * NTRTsim defaults to moving cable anchor points to the edges of
     * a rigid body.
     */
    const static bool stringMoveCablePointAToEdge = true;
    const static bool stringMoveCablePointBToEdge = true;

    /*
     * YAML-encoded structure path.
     */
    std::string topLvlStructurePath;

    /**
     * Boolean flag that enables or disables debugging.
     * All places this flag works in TensegrityModel.cpp can be found
     * by searching through that file for the string "DEBUGGING".
     */
    bool debugging_on = false;

    /**
     * The simplest constructor.
     * This constructor sets debugging_on = false.
     * @param[in] structurePath the path of the YAML-encoded structure
     */
    TensegrityModel(const std::string& structurePath);

    /**
     * Constructor that takes 'debugging' as a parameter.
     * @param[in] structurePath the path of the YAML-encoded structure
     * @param[in] debugging the flag that controls debugging output on/off.
     */
    TensegrityModel(const std::string& structurePath, bool debugging);

    /**
     * Destructor. Deletes controllers, if any were added during setup.
     * Teardown handles everything else.
     */
    virtual ~TensegrityModel();

    /**
     * Create the model. Places the rods and strings into the world
     * that is passed into the simulation. This is triggered
     * automatically when the model is added to the simulation, when
     * tgModel::setup(world) is called (if this model is a child),
     * and when reset is called. Also notifies controllers of setup.
     * @param[in] world the world we're building into
     */
    virtual void setup(tgWorld& world);

    /**
     * Undoes setup. Deletes child models. Called automatically on
     * reset and end of simulation. Notifies controllers of teardown
     */
    virtual void teardown();

    /**
     * Step the model, its children. Notifies controllers of step.
     * @param[in] timeStep the timestep, must be positive.
     */
    virtual void step(double timeStep);

    /**
     * Receives a tgModelVisitor and dispatches itself into the
     * visitor's 'render' function. This model will go to the default
     * tgModel function, which does nothing.
     * @param[in] visitor a tgModelVisitor which will pass this model back
     * to itself
     */
    virtual void onVisit(tgModelVisitor& visitor);

    /**
     * Returns a vector of all muscles for the controllers to work with.
     * @return A vector of all of the muscles
     */
    const std::vector<tgSpringCableActuator*>& getAllActuators() const;

private:
    /**
     * A list of all of the spring cable actuators.
     */
    std::vector<tgSpringCableActuator*> allActuators;

    /*
     * Responsible for adding all the children defined in a structure file, and apply their
     * rotation, scale, offset and translation attributes.
     */
    void addChildren(tgStructure& structure, const std::string& structurePath, tgBuildSpec& spec, const Yam& substructures);

    /*
     * Responsible for adding the child structure defined in the file childStructurePath.
     */
    void addChild(tgStructure& structure, const std::string& parentPath,
        const std::string& childName, const Yam& childStructurePath, tgBuildSpec& spec);

    /*
     * Responsible for applying any rotation attributes for a child structure.
     */
    void addChildRotation(tgStructure& childStructure, const Yam& rotation);

    /*
     * Responsible for applying any scale attributes for a child structure.
     */
    void addChildScale(tgStructure& childStructure, const Yam& scale);

    /*
     * Responsible for applying any offset attributes for a child structure.
     * An offset is a translation defined by an offset multiplied by an offset index.
     */
    void addChildOffset(tgStructure& childStructure, int offsetIndex, const Yam& offset);

    /*
     * Responsible for applying any translation attributes for a child structure.
     */
    void addChildTranslation(tgStructure& childStructure, const Yam& translation);

    /*
     * Responsible for building a structure. This includes adding children, builders, nodes, pairs and bonds.
     */
    void buildStructure(tgStructure& structure, const std::string& structurePath, tgBuildSpec& spec);

    /*
     * Responsible for adding nodes to the structure.
     */
    void addNodes(tgStructure& structure, const Yam& nodes);

    /*
     * Responsible for adding pairs groups to the structure. A pair group is defined by a tag which is used to match
     * pairs with builders that have the same tag.
     */
    void addPairGroups(tgStructure& structure, const Yam& pair_groups);

    /*
     * Responsible for adding bond groups to the structure. Multiple bonds can be nested within a tag or multiple
     * tags can be nested within a bond.
     */
    void addBondGroups(tgStructure& structure, const Yam& bond_groups, tgBuildSpec& spec);

    /*
     * Responsible for adding bonds to the structure.
     * A bond is defined as: 'structureName1/structureName2/.../bond_type'. More than two structure can be bonded
     * together this way.
     */
    void addBonds(tgStructure& structure, const std::string& bonds, const std::string& tags, const Yam& pairs, tgBuildSpec& spec);

    /*
     * Responsible for adding node_node pairs to the structure.
     * A node_node pair is just a pair between two nodes.
     */
    void addNodeNodePairs(tgStructure& structure, const std::string& tags, const Yam& pairs,
        const std::string* childStructure1Name = NULL, const std::string* childStructure2Name = NULL);

    /*
     * Responsible for adding node_edge pairs to the structure.
     * A node_edge pair is a bond between nodes and edges. At least three node to edge pairs must be defined
     * for a valid node_edge bond.
     */
    void addNodeEdgePairs(tgStructure& structure, const std::string& tags, const Yam& pairs,
        const std::string* childStructure1Name, const std::string* childStructure2Name, tgBuildSpec& spec);

    /*
     * Responsible for parsing node edge pairs defined in YAML-encoded structure and populating reference node arrays
     * as well as ligand and receptor arrays (ligand = node, receptor = pair)
     */
    void parseNodeEdgePairs(tgStructure& childStructure1, tgStructure& childStructure2,
        std::vector<btVector3>& structure1RefNodes, std::vector<btVector3>& structure2RefNodes,
        std::vector<tgNode*>& ligands, std::vector< std::pair<tgNode*, tgNode*> >& receptors, const Yam& links);

    /*
     * Responsible for parsing an attachment point, which is either a node or pair and populating the reference node array
     * and the ligand or receptor array.
     */
    void parseAttachmentPoint(tgStructure& structure, const std::string& attachment,  std::vector<btVector3>& refNodes,
        std::vector<tgNode*>& ligands, std::vector< std::pair<tgNode*, tgNode*> >& receptors);

    /*
     * Responsible for applying transformations to the second structure bonded in a node_edge bond so that it is positioned
     * properly to be connected to the first structure.
     */
    void rotateAndTranslate(tgStructure& childStructure2,
        std::vector<btVector3>& structure1RefNodes, std::vector<btVector3>& structure2RefNodes);

    /*
     * Responsible for removing a pair from a structure. Makes sure only pairs that are strings can be removed.
     */
    void removePair(tgStructure& structure, const tgNode* from, const tgNode* to, bool isEdgePair,
        const std::vector<tgBuildSpec::RigidAgent*>& rigidAgents, tgBuildSpec& spec);

    /*
     * Returns a node defined by a path of the form: 'parentStructure.childStructure.nodeName'
     */
    tgNode& getNode(tgStructure& structure, const std::string& nodePath);

    /*
     * Returns a structure defined by a path of the form: 'parentStructure.childStructure'
     */
    tgStructure& getStructure(tgStructure& parentStructure, const std::string& structurePath);

    /*
     * Responsible for adding any builders to the build spec
     */
    void addBuilders(tgBuildSpec& spec, const Yam& builders);

    /*
     * Responsible for adding a builder that uses the tgRod config
     */
    void addRodBuilder(const std::string& builderClass, const std::string& tagMatch, const Yam& parameters, tgBuildSpec& spec);

    /*
     * Responsible for adding a builder that uses the tgBasicActuator config
     */
    void addBasicActuatorBuilder(const std::string& builderClass, const std::string& tagMatch, const Yam& parameters, tgBuildSpec& spec);

    /*
     * Responsible for adding a builder that uses the tgKinematicActuator config
     */
    void addKinematicActuatorBuilder(const std::string& builderClass, const std::string& tagMatch, const Yam& parameters, tgBuildSpec& spec);

    /*
     * Responsible for adding a builder that uses the tgBox config
     */
    void addBoxBuilder(const std::string& builderClass, const std::string& tagMatch, const Yam& parameters, tgBuildSpec& spec);

    /*
     * Responsible for adding a builder that uses the tgSphere config
     */
    void addSphereBuilder(const std::string& builderClass, const std::string& tagMatch, const Yam& parameters, tgBuildSpec& spec);

    /*
     * Ensures YAML node contains only keys from the supplied vector
     */
    void yamlContainsOnly(const Yam& yam, const std::string structurePath, const std::vector<std::string> keys);

    /*
     * Ensures that YAML has all unique keys within each map
     */
    void yamlNoDuplicates(const Yam& yam, const std::string structurePath);

    /**
     * Output debugging information for this model and structure.
     */
    void trace(const tgStructure& structure,
		      const tgStructureInfo& structureInfo, tgModel& model);

};

#endif  // TENSEGRITY_MODEL_H
