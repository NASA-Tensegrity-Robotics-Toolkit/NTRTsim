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
 * @file DuCTTRobotModel.cpp
 * @brief Contains the definition of the members of the class DuCTTModel.
 * $Id$
 */

// This module
#include "DuCTTRobotModel.h"
#include "tgDuCTTHingeInfo.h"
#include "tgPrismaticInfo.h"
#include "tgTouchSensorSphereInfo.h"
#include "tgTouchSensorSphereModel.h"

// The NTRT Core Libary
#include "core/tgLinearString.h"
#include "core/tgRod.h"
#include "core/tgSphere.h"

// The NTRT Creator Libary
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgLinearStringInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgSphereInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"

// The Bullet Physics library
#include "LinearMath/btVector3.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"

// The C++ Standard Library
#include <stdexcept>

DuCTTRobotModel::Config::Config(
    //robot params
    btVector3 startPos,
    btVector3 startRotAxis,
    double startRotAngle,
    //tetra prams
    double triangle_length,
    double duct_distance,
    double duct_height,
    //rod params
    double prismRadius,
    double prismExtent,
    double prismDensity,
    double vertRodRadius,
    double vertDensity,
    double innerRodRadius,
    double innerDensity,
    //sphere tip params
    double tipRad,
    double tipDens,
    double tipFric,
    //string params
    double stiffness,
    double damping,
    double pretension,
    double maxVertStringVel,
    double maxSaddleStringVel,
    double maxStringForce,
    double minStringRestLength
    ) :
m_startPos(startPos),
m_startRotAxis(startRotAxis),
m_startRotAngle(startRotAngle),
m_triangle_length(triangle_length),
m_duct_distance(duct_distance),
m_duct_height(duct_height),
m_prismDensity(prismDensity),
m_prismRadius(prismRadius),
m_prismExtent(prismExtent),
m_vertRodRadius(vertRodRadius),
m_vertDensity(vertDensity),
m_innerRodRadius(innerRodRadius),
m_innerDensity(innerDensity),
m_tipRad(tipRad),
m_tipDens(tipDens),
m_tipFric(tipFric),
m_stiffness(stiffness),
m_damping(damping),
m_pretension(pretension),
m_maxVertStringVel(maxVertStringVel),
m_maxSaddleStringVel(maxSaddleStringVel),
m_maxStringForce(maxStringForce),
m_minStringRestLength(minStringRestLength)
{
}

DuCTTRobotModel::DuCTTRobotModel() :
    m_config(DuCTTRobotModel::Config()),
    tgModel()
{
}

DuCTTRobotModel::DuCTTRobotModel(DuCTTRobotModel::Config &config) :
    m_config(config),
    tgModel()
{
}

DuCTTRobotModel::~DuCTTRobotModel()
{
}

void DuCTTRobotModel::addNodes(tgStructure& tetra,
                            double edge,
                            double distance,
                            double height,
                            double distBtHinges,
                            double distBtNodes)
{
    //Main nodes
    btVector3 bottomRight = btVector3(edge/2.0, distance, 0);
    btVector3 bottomLeft = btVector3(-edge/2.0, distance, 0);

    btVector3 topFront = btVector3(0, distance+height, edge/2.0);
    btVector3 topBack = btVector3(0, distance+height, -edge/2.0);

    //Prismatic nodes
    btVector3 bottomMidRight = btVector3(0.01, distance, 0);
    btVector3 bottomMidLeft = btVector3(-0.01, distance, 0);

    btVector3 topMidFront = btVector3(0, distance+height, 0.01);
    btVector3 topMidBack = btVector3(0, distance+height, -0.01);

    //bottom Hinge nodes
    btVector3 RightFrontSlope = topFront - bottomRight;
    btVector3 LeftFrontSlope = topFront - bottomLeft;
    btVector3 RightBackSlope = topBack - bottomRight;
    btVector3 LeftBackSlope = topBack - bottomLeft;

    btVector3 bottomRightFront = bottomRight + distBtHinges*RightFrontSlope;
    btVector3 bottomLeftFront = bottomLeft + distBtHinges*LeftFrontSlope;
    btVector3 bottomRightBack = bottomRight + distBtHinges*RightBackSlope;
    btVector3 bottomLeftBack = bottomLeft + distBtHinges*LeftBackSlope;
    bottomRight = bottomRight - btVector3(distBtNodes,0,0);
    bottomLeft = bottomLeft + btVector3(distBtNodes,0,0);

    //top Hinge nodes
    btVector3 topRightFront = topFront - distBtHinges*RightFrontSlope;
    btVector3 topLeftFront = topFront - distBtHinges*LeftFrontSlope;
    btVector3 topRightBack = topBack - distBtHinges*RightBackSlope;
    btVector3 topLeftBack = topBack - distBtHinges*LeftBackSlope;
    topFront = topFront - btVector3(0,0,distBtNodes);
    topBack = topBack + btVector3(0,0,distBtNodes);

    if (distance == 0)
    {
        tetra.addNode(bottomRight.x(), bottomRight.y(), bottomRight.z(), "sphere bottom right"); // 0
        tetra.addNode(bottomLeft.x(), bottomLeft.y(), bottomLeft.z(), "sphere bottom left"); // 1
        tetra.addNode(topBack); // 2
        tetra.addNode(topFront); // 3
    }
    else
    {
        tetra.addNode(bottomRight); // 0
        tetra.addNode(bottomLeft); // 1
        tetra.addNode(topBack.x(), topBack.y(), topBack.z(), "sphere top back"); // 2
        tetra.addNode(topFront.x(), topFront.y(), topFront.z(), "sphere top front"); // 3
    }

    tetra.addNode(bottomMidRight); // 4
    tetra.addNode(bottomMidLeft); // 5

    tetra.addNode(topMidBack); // 6
    tetra.addNode(topMidFront); // 7

    tetra.addNode(bottomRightFront); // 8
    tetra.addNode(bottomLeftFront); // 9

    tetra.addNode(bottomRightBack); // 10
    tetra.addNode(bottomLeftBack); // 11

    tetra.addNode(topRightFront); // 12
    tetra.addNode(topLeftFront); // 13

    tetra.addNode(topRightBack); // 14
    tetra.addNode(topLeftBack); // 15
}

void DuCTTRobotModel::addRods(tgStructure& s, int startNode)
{
    std::string tetra = " bottom";
    if (startNode != 0)
        tetra = " top";
    // for one tetra
    //right rods
    s.addPair( startNode+8, startNode+12, "vert rod"+tetra);
    s.addPair( startNode+10, startNode+14, "vert rod"+tetra);

    //left rods
    s.addPair( startNode+9, startNode+13, "vert rod"+tetra);
    s.addPair( startNode+11, startNode+15, "vert rod"+tetra);

    if (startNode == 0)
    {
        //bottom tetra
        // bottom rods
        s.addPair( startNode+0, startNode+4, "prism rod"+tetra);
        s.addPair( startNode+5, startNode+1, "prism rod"+tetra);

        //top rods
        s.addPair( startNode+2, startNode+3, "inner rod"+tetra);
//        s.addPair( startNode+2, startNode+3, "static rod"+tetra);

        s.addPair( startNode+4, startNode+5, "prismatic"+tetra);

        //bottom right hinges
        s.addPair( startNode+0, startNode+8, "hinge"+tetra);
        s.addPair( startNode+0, startNode+10, "hinge"+tetra);

        //bottom left hinges
        s.addPair( startNode+1, startNode+9, "hinge"+tetra);
        s.addPair( startNode+1, startNode+11, "hinge"+tetra);

        //top front hinges
        s.addPair( startNode+3, startNode+12, "hinge3"+tetra);
        s.addPair( startNode+3, startNode+13, "hinge3"+tetra);

        //top back hinges
        s.addPair( startNode+2, startNode+14, "hinge3"+tetra);
        s.addPair( startNode+2, startNode+15, "hinge3"+tetra);
    }
    else
    {
        //top tetra
        // bottom rods
        s.addPair( startNode+0, startNode+1, "inner rod"+tetra);

        //top rods
        s.addPair( startNode+2, startNode+6, "prism rod"+tetra);
        s.addPair( startNode+7, startNode+3, "prism rod"+tetra);

        s.addPair( startNode+6, startNode+7, "prismatic"+tetra);

        //bottom right hinges
        s.addPair( startNode+0, startNode+8, "hinge3"+tetra);
        s.addPair( startNode+0, startNode+10, "hinge3"+tetra);

        //bottom left hinges
        s.addPair( startNode+1, startNode+9, "hinge3"+tetra);
        s.addPair( startNode+1, startNode+11, "hinge3"+tetra);

        //top front hinges
        s.addPair( startNode+3, startNode+12, "hinge2"+tetra);
        s.addPair( startNode+3, startNode+13, "hinge2"+tetra);

        //top back hinges
        s.addPair( startNode+2, startNode+14, "hinge2"+tetra);
        s.addPair( startNode+2, startNode+15, "hinge2"+tetra);
    }
}

void DuCTTRobotModel::addMuscles(tgStructure& s, int topNodesStart)
{
    //vertical strings
    s.addPair(0, topNodesStart+0,  "vert string cluster1");
    s.addPair(1, topNodesStart+1,  "vert string cluster2");
    s.addPair(2, topNodesStart+2,  "vert string cluster3");
    s.addPair(3, topNodesStart+3,  "vert string cluster4");
    
    //saddle strings
    s.addPair(3, topNodesStart+0,  "saddle string cluster5");
    s.addPair(2, topNodesStart+0,  "saddle string cluster6");
    s.addPair(3, topNodesStart+1,  "saddle string cluster7");
    s.addPair(2, topNodesStart+1,  "saddle string cluster8");
}

void DuCTTRobotModel::setup(tgWorld& world)
{
    setupStructure(world);
    setupGhostStructure(world);
    setupVariables();
    
    // Notify controllers that setup has finished.
    notifySetup();
    
    // Actually setup the children
    tgModel::setup(world);
}

void DuCTTRobotModel::setupStructure(tgWorld &world)
{
    // Define the configurations of the rods and strings
    // rodConfigB has density of 0 so it stays fixed in simulator
    const tgRod::Config prismRodConfig(m_config.m_prismRadius, m_config.m_prismDensity);
    const tgRod::Config staticRodConfig(m_config.m_prismRadius, 0);
    const tgRod::Config vertRodConfig(m_config.m_vertRodRadius, m_config.m_vertDensity);
    const tgRod::Config innerRodConfig(m_config.m_innerRodRadius, m_config.m_innerDensity);

    const tgLinearString::Config vertStringConfig(m_config.m_stiffness, m_config.m_damping, m_config.m_pretension,
                                                  false, 0, m_config.m_maxStringForce, m_config.m_maxVertStringVel,
                                                  m_config.m_minStringRestLength, m_config.m_minStringRestLength);
    const tgLinearString::Config saddleStringConfig(m_config.m_stiffness, m_config.m_damping, m_config.m_pretension,
                                                    false, 0, m_config.m_maxStringForce, m_config.m_maxSaddleStringVel,
                                                    m_config.m_minStringRestLength, m_config.m_minStringRestLength);

    const tgPrismatic::Config prismConfig(2, 0, 0.1, m_config.m_prismExtent, 133.45, 1.016, 0.0254);
    const tgPrismatic::Config prismConfig2(1, M_PI/2.0, 0.1, m_config.m_prismExtent, 133.45, 1.016, 0.0254);

    const tgSphere::Config sphereConfig(m_config.m_tipRad, m_config.m_tipDens, m_config.m_tipFric);

    const tgRodHinge::Config hingeConfig(-SIMD_PI, SIMD_PI,2, false, 0.01, 20, 0.2, 0.9, 0.9, 0);
    const tgRodHinge::Config hingeConfig2(-SIMD_PI, SIMD_PI,0, false, 0.01, 20, 0.2, 0.9, 0.9, 0);
    const tgRodHinge::Config hingeConfig3(-SIMD_PI, SIMD_PI,1, false, 0.01, 20, 0.2, 0.9, 0.9, 0);

    // Create a structure that will hold the details of this model
    tgStructure s;

    // Add nodes to the bottom tetrahedron
    addNodes(s, m_config.m_triangle_length, 0, m_config.m_duct_height);

    // Add rods to the bottom tetrahedron
    addRods(s);

    // Add nodes to top tetrahedron
    addNodes(s, m_config.m_triangle_length, m_config.m_duct_distance, m_config.m_duct_height);

    // Add rods to the top tetrahedron
    addRods(s, 16);

    // Add muscles to the structure
    addMuscles(s, 16);

    // Move the structure so it doesn't start in the ground
    s.move(m_config.m_startPos);
    s.addRotation(m_config.m_startPos, m_config.m_startRotAxis, m_config.m_startRotAngle);

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("prism rod", new tgRodInfo(prismRodConfig));
    spec.addBuilder("static rod", new tgRodInfo(staticRodConfig));
    spec.addBuilder("vert rod", new tgRodInfo(vertRodConfig));
    spec.addBuilder("inner rod", new tgRodInfo(innerRodConfig));

    spec.addBuilder("vert string", new tgLinearStringInfo(vertStringConfig));
    spec.addBuilder("saddle string", new tgLinearStringInfo(saddleStringConfig));

    spec.addBuilder("prismatic bottom", new tgPrismaticInfo(prismConfig));
    spec.addBuilder("prismatic top", new tgPrismaticInfo(prismConfig2));
    spec.addBuilder("sphere", new tgSphereInfo(sphereConfig));

    spec.addBuilder("hinge", new tgDuCTTHingeInfo(hingeConfig));
    spec.addBuilder("hinge2", new tgDuCTTHingeInfo(hingeConfig2));
    spec.addBuilder("hinge3", new tgDuCTTHingeInfo(hingeConfig3));

    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);
}

void DuCTTRobotModel::setupGhostStructure(tgWorld &world)
{
    const tgSphere::Config sphereConfig(m_config.m_tipRad, m_config.m_tipDens, m_config.m_tipFric);

    // Create a structure that will hold the details of this model
    tgStructure s;

    // Add nodes to the bottom tetrahedron
    addNodes(s, m_config.m_triangle_length, 0, m_config.m_duct_height);

    // Add nodes to top tetrahedron
    addNodes(s, m_config.m_triangle_length, m_config.m_duct_distance, m_config.m_duct_height);

    // Move the structure so it doesn't start in the ground
    s.move(m_config.m_startPos);
    s.addRotation(m_config.m_startPos, m_config.m_startRotAxis, m_config.m_startRotAngle);

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("sphere", new tgTouchSensorSphereInfo(sphereConfig));

    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);
}

void DuCTTRobotModel::setupVariables()
{
    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control.
    allMuscles = tgCast::filter<tgModel, tgLinearString> (getDescendants());
    allPrisms = tgCast::filter<tgModel, tgPrismatic> (getDescendants());
    allRods = tgCast::filter<tgModel, tgRod> (getDescendants());

    bottomRods = find<tgRod>("rod bottom");
    topRods = find<tgRod>("rod top");
    prismRods = find<tgRod>("prism rod");
    vertMuscles = find<tgLinearString>("vert string");
    saddleMuscles = find<tgLinearString>("saddle string");

    spheres = find<tgSphere>("sphere");
    allTouchSensors = find<tgTouchSensorSphereModel>("sphere");
    bottomSpheres = find<tgSphere>("sphere bottom");
    topSpheres = find<tgSphere>("sphere top");
    bottomTouchSensors = find<tgTouchSensorSphereModel>("sphere bottom");
    topTouchSensors = find<tgTouchSensorSphereModel>("sphere top");

    //attach touch sensors to the appropriate spheres on the actual robot model
    //add the appropriate rods to the touch sensor ignore list
    for (size_t i=0; i<bottomTouchSensors.size(); i++)
    {
        btVector3 offset = bottomTouchSensors[i]->centerOfMass() - bottomSpheres[i]->centerOfMass();
        abstractMarker marker (bottomSpheres[i]->getPRigidBody(), offset, btVector3(255,0,0), 0);
        bottomTouchSensors[i]->addMarker(marker);
        for (size_t j=0; j<bottomRods.size(); j++)
        {
            bottomTouchSensors[i]->addIgnoredObject(bottomRods[j]->getPRigidBody());
        }
    }
    for (size_t i=0; i<topTouchSensors.size(); i++)
    {
        btVector3 offset = topTouchSensors[i]->centerOfMass() - topSpheres[i]->centerOfMass();
        abstractMarker marker (topSpheres[i]->getPRigidBody(), offset, btVector3(255,0,0), 0);
        topTouchSensors[i]->addMarker(marker);
        for (size_t j=0; j<topRods.size(); j++)
        {
            topTouchSensors[i]->addIgnoredObject(topRods[j]->getPRigidBody());
        }
    }

    std::vector<tgPrismatic*> bottomPrisms = find<tgPrismatic>("prismatic bottom");
    if (bottomPrisms.size() == 1)
    {
        m_pBottomPrismatic = bottomPrisms[0];
    }

    std::vector<tgPrismatic*> topPrisms = find<tgPrismatic>("prismatic top");
    if (topPrisms.size() == 1)
    {
        m_pTopPrismatic = topPrisms[0];
    }
}

void DuCTTRobotModel::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of the step so that they can take action
        tgModel::step(dt);  // Step any children
        notifyStep(dt);
    }
}

void DuCTTRobotModel::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

const std::vector<tgLinearString*>& DuCTTRobotModel::getAllMuscles() const
{
    return allMuscles;
}

const std::vector<tgLinearString*>& DuCTTRobotModel::getVertMuscles() const
{
    return vertMuscles;
}

const std::vector<tgLinearString*>& DuCTTRobotModel::getSaddleMuscles() const
{
    return saddleMuscles;
}

const std::vector<tgPrismatic*>& DuCTTRobotModel::getAllPrismatics() const
{
    return allPrisms;
}

const std::vector<tgTouchSensorSphereModel*>& DuCTTRobotModel::getAllTouchSensors() const
{
    return allTouchSensors;
}

tgPrismatic* DuCTTRobotModel::getBottomPrismatic()
{
    return m_pBottomPrismatic;
}

tgPrismatic* DuCTTRobotModel::getTopPrismatic()
{
    return m_pTopPrismatic;
}

btVector3 DuCTTRobotModel::getCOM()
{
    std::vector<tgRod*> rods = find<tgRod>("rod");
    assert(!rods.empty());

    btVector3 tetraCenterOfMass(0, 0, 0);
    double tetraMass = 0.0;
    for (std::size_t i = 0; i < rods.size(); i++) {
        const tgRod* const rod = rods[i];
        assert(rod != NULL);
        const double rodMass = rod->mass();
        const btVector3 rodCenterOfMass = rod->centerOfMass();
        tetraCenterOfMass += rodCenterOfMass * rodMass;
        tetraMass += rodMass;
    }

    assert(tetraMass > 0.0);
    tetraCenterOfMass /= tetraMass;

    return tetraCenterOfMass;
}

btVector3 DuCTTRobotModel::getTetraCOM(bool bottom)
{
    std::vector<tgRod*> rods;
    if (bottom)
    {
        rods = find<tgRod>("rod bottom");
    }
    else
    {
        rods = find<tgRod>("rod top");
    }
    assert(!rods.empty());

    btVector3 tetraCenterOfMass(0, 0, 0);
    double tetraMass = 0.0;
    for (std::size_t i = 0; i < rods.size(); i++) {
        const tgRod* const rod = rods[i];
        assert(rod != NULL);
        const double rodMass = rod->mass();
        const btVector3 rodCenterOfMass = rod->centerOfMass();
        tetraCenterOfMass += rodCenterOfMass * rodMass;
        tetraMass += rodMass;
    }

    assert(tetraMass > 0.0);
    tetraCenterOfMass /= tetraMass;

    return tetraCenterOfMass;
}

void DuCTTRobotModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();

    allMuscles.clear();
    vertMuscles.clear();
    saddleMuscles.clear();
    allPrisms.clear();
    allRods.clear();
    bottomRods.clear();
    topRods.clear();
    prismRods.clear();
    spheres.clear();
    bottomSpheres.clear();
    topSpheres.clear();
    allTouchSensors.clear();
    bottomSpheres.clear();
    topTouchSensors.clear();
}
